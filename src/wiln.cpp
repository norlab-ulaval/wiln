#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <mutex>
#include "wiln.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

WilnNode::WilnNode() : Node("wiln_node"), currentState(State::IDLE)
{
    // TODO: Add a publisher for the state
    initParameters();
    initSubscribers();
    initPublishers();
    initServices();
    initClients();
    initTimers();
}

void WilnNode::initParameters()
{
    this->declare_parameter("odom_topic", "odom");
    this->declare_parameter("distance_between_waypoints", 0.05);
    this->declare_parameter("angle_between_waypoints", 0.1);
    this->declare_parameter("trajectory_speed", 1.5);
    this->declare_parameter("smoothing_window_size", 9);
    updateParameters();
}

void WilnNode::updateParameters()
{
    this->get_parameter("odom_topic", odomTopic);
    this->get_parameter("distance_between_waypoints", distanceBetweenWaypoints);
    this->get_parameter("angle_between_waypoints", angleBetweenWaypoints);
    this->get_parameter("trajectory_speed", trajectorySpeed);
    this->get_parameter("smoothing_window_size", smoothingWindowSize);
}

void WilnNode::initSubscribers()
{
    odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
        odomTopic, 10, std::bind(&WilnNode::odomCallback, this, std::placeholders::_1)
    );
    // trajectoryResultSubscription = this->create_subscription<norlab_controllers_msgs::action::FollowPath::Result>(
    //         "follow_path/result", 1000, std::bind(&WilnNode::trajectoryResultCallback, this, std::placeholders::_1)
    // );
}

void WilnNode::initPublishers()
{
    auto publisher_qos = rclcpp::QoS(10);
    publisher_qos.transient_local();
    plannedTrajectoryPublisher = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory", publisher_qos);
    realTrajectoryPublisher = this->create_publisher<nav_msgs::msg::Path>("real_trajectory", publisher_qos);
    statePublisher = this->create_publisher<std_msgs::msg::UInt8>("state", 1);

    publishPlannedTrajectory();
    publishRealTrajectory();
}

void WilnNode::initServices()
{
    startRecordingService = this->create_service<std_srvs::srv::Empty>(
        "start_recording", std::bind(&WilnNode::startRecordingServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    stopRecordingService = this->create_service<std_srvs::srv::Empty>(
        "stop_recording", std::bind(&WilnNode::stopRecordingServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    saveMapTrajService = this->create_service<wiln::srv::SaveMapTraj>(
        "save_map_traj", std::bind(&WilnNode::saveLTRServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    loadMapTrajService = this->create_service<wiln::srv::LoadMapTraj>(
        "load_map_traj", std::bind(&WilnNode::loadLTRServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    loadMapTrajFromEndService = this->create_service<wiln::srv::LoadMapTraj>(
        "load_map_traj_from_end", std::bind(&WilnNode::loadLTRFromEndServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    playLoopService = this->create_service<wiln::srv::PlayLoop>(
        "play_loop", std::bind(&WilnNode::playLoopServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    playLineService = this->create_service<std_srvs::srv::Empty>(
        "play_line", std::bind(&WilnNode::playLineServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    cancelTrajectoryService = this->create_service<std_srvs::srv::Empty>(
        "cancel_trajectory", std::bind(&WilnNode::cancelTrajectoryServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    smoothTrajectoryService = this->create_service<std_srvs::srv::Empty>(
        "smooth_trajectory", std::bind(&WilnNode::smoothTrajectoryServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    clearTrajectoryService = this->create_service<std_srvs::srv::Empty>(
        "clear_trajectory", std::bind(&WilnNode::clearTrajectoryServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    reverseTrajectoryService = this->create_service<std_srvs::srv::Empty>(
        "reverse_trajectory", std::bind(&WilnNode::reverseTrajectoryServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    flipTrajectoryService = this->create_service<std_srvs::srv::Empty>(
        "flip_trajectory", std::bind(&WilnNode::flipTrajectoryServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void WilnNode::initClients()
{
    enableMappingClient = this->create_client<std_srvs::srv::Empty>("/mapping/enable_mapping");
    disableMappingClient = this->create_client<std_srvs::srv::Empty>("/mapping/disable_mapping");
    saveMapClient = this->create_client<norlab_icp_mapper_ros::srv::SaveMap>("/mapping/save_map");
    loadMapClient = this->create_client<norlab_icp_mapper_ros::srv::LoadMap>("/mapping/load_map");
    followPathClient = rclcpp_action::create_client<norlab_controllers_msgs::action::FollowPath>(this, "/follow_path");
}

void WilnNode::initTimers()
{
    stateTimer = this->create_wall_timer(500ms, std::bind(&WilnNode::publishState, this));
    updateTimer = this->create_wall_timer(2s, std::bind(&WilnNode::updateParameters, this));
}

void WilnNode::odomCallback(const nav_msgs::msg::Odometry &odomMsg)
{
    robotPoseLock.lock();
    robotPose.header = odomMsg.header;
    robotPose.pose = odomMsg.pose.pose;
    robotPoseLock.unlock();

    if (currentState == State::RECORDING)
    {
        updatePlannedTrajectory(robotPose);
    }
    else if (currentState == State::PLAYING)
    {
        updateRealTrajectory(robotPose);
    }
}

void WilnNode::updatePlannedTrajectory(const geometry_msgs::msg::PoseStamped &currentPose)
{
    // Create new trajectory if none exists
    if (plannedTrajectory.poses.empty())
    {
        plannedTrajectory.header.frame_id = currentPose.header.frame_id;
        plannedTrajectory.header.stamp = this->now();
        plannedTrajectory.poses.push_back(currentPose);
    }
    else
    {
        // Add points if the robot moved enough
        geometry_msgs::msg::Pose lastPose = plannedTrajectory.poses.back().pose;
        auto [diff_lin, diff_ang] = diffBetweenPoses(lastPose, currentPose.pose);
        if (diff_lin >= distanceBetweenWaypoints || std::fabs(diff_ang) > angleBetweenWaypoints)
        {
            plannedTrajectory.poses.push_back(currentPose);
        }
    }

    publishPlannedTrajectory();
}

void WilnNode::updateRealTrajectory(const geometry_msgs::msg::PoseStamped &currentPose)
{
    // Create new trajectory if none exists
    if (realTrajectory.poses.empty())
    {
        realTrajectory.header.frame_id = currentPose.header.frame_id;
        realTrajectory.header.stamp = this->now();
        realTrajectory.poses.push_back(currentPose);
    }
    else
    {
        // Add points if the robot moved enough
        geometry_msgs::msg::Pose lastPose = realTrajectory.poses.back().pose;
        auto [diff_lin, diff_ang] = diffBetweenPoses(lastPose, currentPose.pose);
        if (diff_lin >= distanceBetweenWaypoints || std::fabs(diff_ang) > angleBetweenWaypoints)
        {
            realTrajectory.poses.push_back(currentPose);
        }
    }

    publishRealTrajectory();
}

void WilnNode::startRecordingServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    switch (currentState)
    {
        case State::IDLE:
        {
            currentState = State::RECORDING;
            auto enableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
            enableMappingClient->async_send_request(enableMappingRequest);
            RCLCPP_INFO(this->get_logger(), "Recording started.");
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being recorded.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot start recording, trajectory is currently being played.");
            break;
        }
    }
}

void WilnNode::stopRecordingServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    switch (currentState)
    {
        case State::IDLE:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot stop recording, no trajectory is being recorded.");
            break;
        }
        case State::RECORDING:
        {
            currentState = State::IDLE;
            auto disableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
            disableMappingClient->async_send_request(disableMappingRequest);
            RCLCPP_INFO(this->get_logger(), "Recording stopped.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot stop recording, trajectory is currently being played.");
            break;
        }
    }
}

void WilnNode::clearTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    switch (currentState)
    {   
        case State::IDLE:
        {
            RCLCPP_INFO(this->get_logger(), "Clearing trajectory.");
            plannedTrajectory.poses.clear();
            publishPlannedTrajectory();
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot clear trajectory while recording.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot clear trajectory while playing.");
            break;
        }
    }
}

void WilnNode::reverseTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    switch (currentState)
    {   
        case State::IDLE:
        {
            RCLCPP_INFO(this->get_logger(), "Reversing trajectory.");
            plannedTrajectory = reversePath(plannedTrajectory);
            publishPlannedTrajectory();
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot reverse trajectory while recording.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot reverse trajectory while playing.");
            break;
        }
    }
}

void WilnNode::flipTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    switch (currentState)
    {   
        case State::IDLE:
        {
            RCLCPP_INFO(this->get_logger(), "Flipping trajectory.");
            plannedTrajectory = flipPath(plannedTrajectory);
            publishPlannedTrajectory();
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot flip trajectory while recording.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot flip trajectory while playing.");
            break;
        }
    }
}

void WilnNode::smoothTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Smoothing trajectory.");
    plannedTrajectory = smoothPathLowPass(plannedTrajectory, smoothingWindowSize);
    publishPlannedTrajectory();
}

void WilnNode::cancelTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    switch (currentState)
    {   
        case State::IDLE:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel trajectory, no trajectory is being played.");
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel trajectory, trajectory is currently being recorded.");
            break;
        }
        case State::PLAYING:
        {
            currentState = State::IDLE;
            followPathClient->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "Cancelled trajectory.");
            break;
        }
    }
}

void WilnNode::saveLTRServiceCallback(const std::shared_ptr<wiln::srv::SaveMapTraj::Request> req, std::shared_ptr<wiln::srv::SaveMapTraj::Response> res)
{
    // TODO: force save to same working repository as mapper
    using namespace std::chrono_literals;
    auto saveMapRequest = std::make_shared<norlab_icp_mapper_ros::srv::SaveMap::Request>();
    std::string mapName = req->file_name.data.substr(0, req->file_name.data.rfind('.')) + ".vtk";
    saveMapRequest->map_file_name.data = mapName;
    //        auto saveMapFuture = saveMapClient->async_send_request(saveMapRequest);
    //        std::shared_ptr<norlab_icp_mapper_ros::srv::SaveMap::Request> request = std::make_shared<norlab_icp_mapper_ros::srv::SaveMap::Request>();
    RCLCPP_INFO(this->get_logger(), "calling /save_map");
    norlab_icp_mapper_ros::srv::SaveMap::Response response = rclcpp::call_service<norlab_icp_mapper_ros::srv::SaveMap>("/mapping/save_map", saveMapRequest);
    RCLCPP_INFO(this->get_logger(), "/save_map done");

    std::rename(mapName.c_str(), req->file_name.data.c_str());
    std::ofstream ltrFile(req->file_name.data, std::ios::app);

    ltrFile << TRAJECTORY_DELIMITER << std::endl;
    ltrFile << "frame_id : " << plannedTrajectory.header.frame_id << std::endl;

    for (auto pose : plannedTrajectory.poses)
    {
        ltrFile << pose.pose.position.x << ","
                << pose.pose.position.y << ","
                << pose.pose.position.z << ","
                << pose.pose.orientation.x << ","
                << pose.pose.orientation.y << ","
                << pose.pose.orientation.z << ","
                << pose.pose.orientation.w << std::endl;
    }

    ltrFile.close();
    RCLCPP_INFO(this->get_logger(), "LTR file succesfully saved");
}

void WilnNode::loadLTRServiceCallback(const std::shared_ptr<wiln::srv::LoadMapTraj::Request> req, std::shared_ptr<wiln::srv::LoadMapTraj::Response> res)
{
	auto disableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
	disableMappingClient->async_send_request(disableMappingRequest);
	RCLCPP_INFO(this->get_logger(), "Disabled mapping before loading the LTR file");
    loadLTR(req->file_name.data, false);
    return;
}

void WilnNode::loadLTRFromEndServiceCallback(const std::shared_ptr<wiln::srv::LoadMapTraj::Request> req, std::shared_ptr<wiln::srv::LoadMapTraj::Response> res)
{
	auto disableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
	disableMappingClient->async_send_request(disableMappingRequest);
	RCLCPP_INFO(this->get_logger(), "Disabled mapping before loading the LTR file");
    loadLTR(req->file_name.data, true);
    return;
}

void WilnNode::loadLTR(std::string fileName, bool fromEnd)
{
    using namespace std::chrono_literals;
    plannedTrajectory.poses.clear();
    std::ofstream mapFile("/tmp/map.vtk");
    std::ifstream ltrFile(fileName);
    std::string line;
    std::string pathFrameId;
    bool parsingMap = true;

    // Base pose
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();

    while (std::getline(ltrFile, line))
    {
        if (parsingMap)
        {
            if (line.find(TRAJECTORY_DELIMITER) != std::string::npos)
            {
                std::getline(ltrFile, line);
                pathFrameId = line.substr(FRAME_ID_START_POSITION);
                plannedTrajectory.header.frame_id = pathFrameId;
                plannedTrajectory.header.stamp = this->now();
                parsingMap = false;
            }
            else
            {
                mapFile << line << std::endl;
            }
        }
        else
        {
            if (line.find("changing direction") != std::string::npos)
            {
                continue; // Ignore
            } 

            std::stringstream ss(line);
            std::string token;
            std::getline(ss, token, ',');
            pose.pose.position.x = std::stod(token);
            std::getline(ss, token, ',');
            pose.pose.position.y = std::stod(token);
            std::getline(ss, token, ',');
            pose.pose.position.z = std::stod(token);
            std::getline(ss, token, ',');
            pose.pose.orientation.x = std::stod(token);
            std::getline(ss, token, ',');
            pose.pose.orientation.y = std::stod(token);
            std::getline(ss, token, ',');
            pose.pose.orientation.z = std::stod(token);
            std::getline(ss, token);
            pose.pose.orientation.w = std::stod(token);
            pose.header.frame_id = pathFrameId;
            plannedTrajectory.poses.push_back(pose);
        }
    }
    ltrFile.close();
    mapFile.close();

    auto loadMapRequest = std::make_shared<norlab_icp_mapper_ros::srv::LoadMap::Request>();
    loadMapRequest->map_file_name.data = "/tmp/map.vtk";

    int poseIndex = fromEnd ? 0 : plannedTrajectory.poses.size() - 1;
    loadMapRequest->pose= plannedTrajectory.poses[poseIndex].pose;
    loadMapClient->async_send_request(loadMapRequest);

    // auto loadMapFuture = loadMapClient->async_send_request(loadMapRequest);
    RCLCPP_INFO(this->get_logger(), "calling /load_map");
    norlab_icp_mapper_ros::srv::LoadMap::Response response = rclcpp::call_service<norlab_icp_mapper_ros::srv::LoadMap>("/mapping/load_map", loadMapRequest);
    RCLCPP_INFO(this->get_logger(), "/load_map done");

    std::remove("/tmp/map.vtk");

    publishPlannedTrajectory();
}

void WilnNode::publishPlannedTrajectory()
{
    plannedTrajectoryPublisher->publish(plannedTrajectory);
}

void WilnNode::publishRealTrajectory()
{
    realTrajectoryPublisher->publish(realTrajectory);
}

void WilnNode::publishState()
{
    std_msgs::msg::UInt8 stateMsg;
    stateMsg.data = static_cast<uint8_t>(currentState);
    statePublisher->publish(stateMsg);
}

void WilnNode::playLineServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    if (plannedTrajectory.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cannot play an empty trajectory.");
        return;
    }

    switch (currentState)
    {
        case State::IDLE:
        {
            RCLCPP_WARN(this->get_logger(), "Playing line.");
            playLine();
            currentState = State::PLAYING;
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play line while recording.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being played.");
            break;
        }
    }
}

void WilnNode::playLine()
{
    robotPoseLock.lock();
    nav_msgs::msg::Path lineTrajectory(plannedTrajectory);

    auto [lin_dist_start, ang_dist_start] = diffBetweenPoses(robotPose.pose, lineTrajectory.poses.front().pose);
    auto [lin_dist_end, ang_dist_end] = diffBetweenPoses(robotPose.pose, lineTrajectory.poses.back().pose);
    RCLCPP_INFO(this->get_logger(), " -> Distance from start: %f", lin_dist_start);
    RCLCPP_INFO(this->get_logger(), " -> Distance from end: %f", lin_dist_end);

    // Reverse trajectory if robot is closer to end
    if (lin_dist_end < lin_dist_start + 2.0)
    {
        RCLCPP_INFO(this->get_logger(), "Reversing trajectory.");
        lineTrajectory = reversePath(lineTrajectory);
    }

    // Flip trajectory if angle_diff is larger than M_PI/2
    auto [lin_dist_final, ang_dist_final] = diffBetweenPoses(robotPose.pose, lineTrajectory.poses.front().pose);
    RCLCPP_INFO(this->get_logger(), " -> Angle error: %f", ang_dist_final);
    if (std::fabs(ang_dist_final) > M_PI/2)
    {
        RCLCPP_INFO(this->get_logger(), "Flipping trajectory.");
        lineTrajectory = flipPath(lineTrajectory);
    }

    robotPoseLock.unlock();
    realTrajectory.poses.clear();

    plannedTrajectory = lineTrajectory;
    publishPlannedTrajectory();
    sendFollowPathAction(lineTrajectory);
}

void WilnNode::playLoopServiceCallback(const std::shared_ptr<wiln::srv::PlayLoop::Request> req, std::shared_ptr<wiln::srv::PlayLoop::Response> res)
{
    if (plannedTrajectory.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cannot play an empty trajectory.");
        return;
    }

    switch (currentState)
    {
        case State::IDLE:
        {
            RCLCPP_WARN(this->get_logger(), "Playing loop %d times.", req->nb_loops.data);
            playLoop(req->nb_loops.data);
            currentState = State::PLAYING;
            break;
        }
        case State::RECORDING:
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play loop while recording.");
            break;
        }
        case State::PLAYING:
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being played.");
            break;
        }
    }
}

void WilnNode::playLoop(int nbLoops)
{
    robotPoseLock.lock();
    nav_msgs::msg::Path loopTrajectory;

    // Remove overlapping poses
    auto cleanTrajectory = removePathOverlap(plannedTrajectory);
    RCLCPP_INFO(this->get_logger(), "Removed %ld overlapping points at the start.", plannedTrajectory.poses.size() - cleanTrajectory.poses.size());

    // Repeat trajectory X times
    for (int i = 0; i < nbLoops; ++i)
    {
        loopTrajectory.poses.insert(loopTrajectory.poses.end(), cleanTrajectory.poses.begin(), cleanTrajectory.poses.end());
    }

    robotPoseLock.unlock();
    realTrajectory.poses.clear();

    sendFollowPathAction(loopTrajectory);
}

void WilnNode::sendFollowPathAction(nav_msgs::msg::Path &path)
{
    auto disableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
    disableMappingClient->async_send_request(disableMappingRequest);

    // TODO: validate action call
    auto goal_msg = norlab_controllers_msgs::action::FollowPath::Goal();
    goal_msg.follower_options.init_mode.data = 1; // init_mode = 1 : continue
    goal_msg.follower_options.velocity.data = trajectorySpeed;
    goal_msg.path = path;

    auto send_goal_options = rclcpp_action::Client<norlab_controllers_msgs::action::FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&WilnNode::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&WilnNode::trajectoryFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&WilnNode::trajectoryResultCallback, this, std::placeholders::_1);
    followPathClient->async_send_goal(goal_msg, send_goal_options);
}

void WilnNode::goalResponseCallback(const rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::SharedPtr &trajectoryGoalHandle)
{
    if (!trajectoryGoalHandle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void WilnNode::trajectoryFeedbackCallback(rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::SharedPtr,
                                const std::shared_ptr<const norlab_controllers_msgs::action::FollowPath::Feedback> feedback)
{
    // TODO: program feedback callback
    return;
}

void WilnNode::trajectoryResultCallback(const rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::WrappedResult &trajectory_result)
{
    if (trajectory_result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully reached goal!");
        RCLCPP_INFO(this->get_logger(), "--------------------------");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Trajectory goal was not reached...");
    }

    currentState = State::IDLE;
}
