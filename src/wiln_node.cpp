#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <mutex>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

#include <wiln/srv/save_map_traj.hpp>
#include <wiln/srv/load_map_traj.hpp>
#include <wiln/srv/play_loop.hpp>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>
#include <service_caller/ServiceCaller.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include <norlab_controllers_msgs/msg/directional_path.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <norlab_controllers_msgs/action/follow_path.hpp>

tf2::Quaternion HALF_TURN_ROTATION(0.0, 0.0, 1.0, 0.0);

class WilnNode : public rclcpp::Node
{
public:
    WilnNode():
            Node("wiln_node")
    {
        using FollowPath = norlab_controllers_msgs::action::FollowPath;

        // TODO: Create service clients to call enable_mapping, disable_mapping, save_map and load_map

        startRecordingService = this->create_service<std_srvs::srv::Empty>("start_recording",
                                                                           std::bind(&WilnNode::startRecordingServiceCallback, this, std::placeholders::_1,
                                                                                     std::placeholders::_2));
        stopRecordingService = this->create_service<std_srvs::srv::Empty>("stop_recording",
                                                                          std::bind(&WilnNode::stopRecordingServiceCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        saveMapTrajService = this->create_service<wiln::srv::SaveMapTraj>("save_map_traj",
                                                                          std::bind(&WilnNode::saveLTRServiceCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        loadMapTrajService = this->create_service<wiln::srv::LoadMapTraj>("load_map_traj",
                                                                          std::bind(&WilnNode::loadLTRServiceCallback, this, std::placeholders::_1,
                                                                                    std::placeholders::_2));
        loadMapTrajFromEndService = this->create_service<wiln::srv::LoadMapTraj>("load_map_traj_from_end",
                                                                                 std::bind(&WilnNode::loadLTRFromEndServiceCallback, this, std::placeholders::_1,
                                                                                           std::placeholders::_2));
        playLoopService = this->create_service<wiln::srv::PlayLoop>("play_loop",
                                                                    std::bind(&WilnNode::playLoopTrajectoryServiceCallback, this, std::placeholders::_1,
                                                                              std::placeholders::_2));
        playLineService = this->create_service<std_srvs::srv::Empty>("play_line",
                                                                     std::bind(&WilnNode::playLineTrajectoryServiceCallback, this, std::placeholders::_1,
                                                                               std::placeholders::_2));
        cancelTrajectoryService = this->create_service<std_srvs::srv::Empty>("cancel_trajectory",
                                                                             std::bind(&WilnNode::cancelTrajectoryServiceCallback, this, std::placeholders::_1,
                                                                                       std::placeholders::_2));
        smoothTrajectoryService = this->create_service<std_srvs::srv::Empty>("smooth_trajectory",
                                                                             std::bind(&WilnNode::smoothTrajectoryServiceCallback, this, std::placeholders::_1,
                                                                                       std::placeholders::_2));
        clearTrajectoryService = this->create_service<std_srvs::srv::Empty>("clear_trajectory",
                                                                            std::bind(&WilnNode::clearTrajectoryServiceCallback, this, std::placeholders::_1,
                                                                                      std::placeholders::_2));

        enableMappingClient = this->create_client<std_srvs::srv::Empty>("enable_mapping");
        disableMappingClient = this->create_client<std_srvs::srv::Empty>("disable_mapping");
        saveMapClient = this->create_client<norlab_icp_mapper_ros::srv::SaveMap>("save_map");
        loadMapClient = this->create_client<norlab_icp_mapper_ros::srv::LoadMap>("load_map");

        followPathClient = rclcpp_action::create_client<norlab_controllers_msgs::action::FollowPath>(this, "follow_path");

        odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("odom_in", 1000,
                                                                              std::bind(&WilnNode::odomCallback, this,
                                                                                        std::placeholders::_1));
        commandedVelocitySubscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_in", 1000,
                                                                                             std::bind(&WilnNode::commandVelocityCallback, this,
                                                                                                       std::placeholders::_1));
        plannedTrajectorySubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_in", 1000,
                                                                                                   std::bind(&WilnNode::plannedTrajectoryCallback, this,
                                                                                                             std::placeholders::_1));
        realTrajectorySubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_in", 1000,
                                                                                                std::bind(&WilnNode::realTrajectoryCallback, this,
                                                                                                          std::placeholders::_1));
//        trajectoryResultSubscription = this->create_subscription<norlab_controllers_msgs::action::FollowPath::Result>("follow_path/result", 1000,
//                                                                                                std::bind(&WilnNode::trajectoryResultCallback, this,
//                                                                                                          std::placeholders::_1));
        auto publisher_qos = rclcpp::QoS(10);
        publisher_qos.transient_local();
        plannedTrajectoryPublisher = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory", publisher_qos);
        realTrajectoryPublisher = this->create_publisher<nav_msgs::msg::Path>("real_trajectory", publisher_qos);

        statusPublisher = this->create_publisher<std_msgs::msg::String>("status", publisher_qos);

        drivingForward.store(true);
        lastDrivingDirection.store(true);
    }

private:
    bool playing;
    bool recording;
    std::atomic_bool drivingForward;
    norlab_controllers_msgs::msg::PathSequence plannedTrajectory;
    norlab_controllers_msgs::msg::PathSequence realTrajectory;
    geometry_msgs::msg::Pose robotPose;
    std::mutex robotPoseLock;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startRecordingService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stopRecordingService;
    rclcpp::Service<wiln::srv::SaveMapTraj>::SharedPtr saveMapTrajService;
    rclcpp::Service<wiln::srv::LoadMapTraj>::SharedPtr loadMapTrajService;
    rclcpp::Service<wiln::srv::LoadMapTraj>::SharedPtr loadMapTrajFromEndService;
    rclcpp::Service<wiln::srv::PlayLoop>::SharedPtr playLoopService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr playLineService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancelTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr smoothTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearTrajectoryService;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr enableMappingClient;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr disableMappingClient;
    rclcpp::Client<norlab_icp_mapper_ros::srv::SaveMap>::SharedPtr saveMapClient;
    rclcpp::Client<norlab_icp_mapper_ros::srv::LoadMap>::SharedPtr loadMapClient;
    std::atomic_bool lastDrivingDirection;

    const int FRAME_ID_START_POSITION = 11; // length of string : "frame_id : "

    const std::string TRAJECTORY_DELIMITER = "#############################";
    float delayBetweenWaypoints;

    float trajectorySpeed;
    int lowPassFilterWindowSize;

    //    const std::map<int8_t, std::string> FOLLOW_PATH_RESULTS = {
//            { 0u, "RESULT_STATUS_STOPPED_BY_SUPERVISOR" },
//            { 1u, "RESULT_STATUS_UNKNOWN" },
//            { 2u, "RESULT_STATUS_OBSTACLE" },
//            { 3u, "RESULT_STATUS_SUCCESS" },
//            { 4u, "RESULT_STATUS_ABORTED" },
//            { 5u, "RESULT_STATUS_INTERNAL_ERROR" },
//            { 6u, "RESULT_STATUS_SLAM_FAIL" },
//            { 7u, "RESULT_STATUS_TF_FAIL" },
//            { 8u, "RESULT_STATUS_PATH_LOST" },
//            { 9u, "RESULT_STATUS_TIMEOUT" },
//    };

    rclcpp_action::Client<norlab_controllers_msgs::action::FollowPath>::SharedPtr followPathClient;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commandedVelocitySubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr plannedTrajectorySubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr realTrajectorySubscription;
//    rclcpp::Subscription<norlab_controllers_msgs::action::FollowPath::Result>::SharedPtr trajectoryResultSubscription;


    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plannedTrajectoryPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr realTrajectoryPublisher;
    rclcpp::Publisher<std_msgs::msg::Path>::SharedPtr statusPublisher;


    void odomCallback(const nav_msgs::msg::Odometry& odomIn)
    {
        robotPoseLock.lock();
        robotPose = odomIn.pose.pose;
        robotPoseLock.unlock();
    }

    void commandVelocityCallback(const geometry_msgs::msg::Twist& commandedVelocity)
    {
        drivingForward.store(commandedVelocity.linear.x >= 0.0);
    }

    float computeAngleBetweenPoses(const geometry_msgs::msg::PoseStamped& firstPose, const geometry_msgs::msg::PoseStamped& secondPose)
    {
        float firstPoseAngle = extractYawFromQuaternion(firstPose.pose.orientation);
        float secondPoseAngle = extractYawFromQuaternion(secondPose.pose.orientation);
        float pathAngle = secondPoseAngle - firstPoseAngle;
        if(pathAngle > M_PI)
        {
            pathAngle -= 2.0 * M_PI;
        }
        else if(pathAngle < -M_PI)
        {
            pathAngle += 2 * M_PI;
        }
        return pathAngle;
    }

    void plannedTrajectoryCallback(const geometry_msgs::msg::PoseStamped& poseStamped)
    {
        if(recording)
        {
            if(plannedTrajectory.paths.empty())
            {
                plannedTrajectory.header.frame_id = poseStamped.header.frame_id;
                plannedTrajectory.header.stamp = this->now();
                norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
                directionalPath.header.stamp = this->now();
                directionalPath.forward = drivingForward.load();
                directionalPath.poses.push_back(poseStamped);
                plannedTrajectory.paths.push_back(directionalPath);
                publishPlannedTrajectory();
            }
            else if(std::fabs(computeAngleBetweenPoses(plannedTrajectory.paths.back().poses.back(), poseStamped)) > 0.5)
            {
                norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
                directionalPath.header.stamp = this->now();
                directionalPath.forward = drivingForward.load();
                geometry_msgs::msg::PoseStamped rotatedPose = poseStamped;
                rotatedPose.pose.position = plannedTrajectory.paths.back().poses.back().pose.position;
                directionalPath.poses.push_back(rotatedPose);
                plannedTrajectory.paths.push_back(directionalPath);
                publishPlannedTrajectory();
            }
            else if(lastDrivingDirection.load() != drivingForward.load())
            {
                norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
                directionalPath.header.stamp = this->now();
                directionalPath.forward = drivingForward.load();
                directionalPath.poses.push_back(poseStamped);
                plannedTrajectory.paths.push_back(directionalPath);
                publishPlannedTrajectory();
            }
            else
            {
                geometry_msgs::msg::PoseStamped lastPose = plannedTrajectory.paths.back().poses.back();
                double distance = std::sqrt(std::pow(poseStamped.pose.position.x - lastPose.pose.position.x, 2) +
                                            std::pow(poseStamped.pose.position.y - lastPose.pose.position.y, 2) +
                                            std::pow(poseStamped.pose.position.z - lastPose.pose.position.z, 2));
                if(distance >= 0.05)
                {
                    plannedTrajectory.paths.back().poses.push_back(poseStamped);
                    publishPlannedTrajectory();
                }
            }
            lastDrivingDirection.store(drivingForward.load());
        }
    }

    void realTrajectoryCallback(const geometry_msgs::msg::PoseStamped& poseStamped)
    {
        if(playing)
        {
            if(realTrajectory.paths.empty())
            {
                realTrajectory.header.frame_id = poseStamped.header.frame_id;
                realTrajectory.header.stamp = this->now();
                norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
                directionalPath.header.stamp = this->now();
                directionalPath.forward = drivingForward.load();
                directionalPath.poses.push_back(poseStamped);
                realTrajectory.paths.push_back(directionalPath);
                publishRealTrajectory();
            }
//            else if(std::fabs(computeAngleBetweenPoses(realTrajectory.paths.back().poses.back(), poseStamped)) > 0.5)
//            {
//                norlab_controllers_msgs::msg::DirectionalPath directionalPath;
//                directionalPath.header.frame_id = poseStamped.header.frame_id;
//                directionalPath.header.stamp = this->now();
//                directionalPath.forward = drivingForward.load();
//                geometry_msgs::msg::PoseStamped rotatedPose = poseStamped;
//                rotatedPose.pose.position = realTrajectory.paths.back().poses.back().pose.position;
//                directionalPath.poses.push_back(rotatedPose);
//                realTrajectory.paths.push_back(directionalPath);
//                publishRealTrajectory();
//            }
            else if(lastDrivingDirection.load() != drivingForward.load())
            {
                norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
                directionalPath.header.stamp = this->now();
                directionalPath.forward = drivingForward.load();
                directionalPath.poses.push_back(poseStamped);
                realTrajectory.paths.push_back(directionalPath);
                publishRealTrajectory();
            }
            else
            {
                geometry_msgs::msg::PoseStamped lastPose = realTrajectory.paths.back().poses.back();
                double distance = std::sqrt(std::pow(poseStamped.pose.position.x - lastPose.pose.position.x, 2) +
                                            std::pow(poseStamped.pose.position.y - lastPose.pose.position.y, 2) +
                                            std::pow(poseStamped.pose.position.z - lastPose.pose.position.z, 2));
                if(distance >= 0.05)
                {
                    realTrajectory.paths.back().poses.push_back(poseStamped);
                    publishRealTrajectory();
                }
            }
            lastDrivingDirection.store(drivingForward.load());
        }
    }

    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::SharedPtr& trajectoryGoalHandle)
    {
        if(!trajectoryGoalHandle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void trajectoryFeedbackCallback(rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::SharedPtr,
                                    const std::shared_ptr<const norlab_controllers_msgs::action::FollowPath::Feedback> feedback)
    {
        //TODO: program feedback callback
        return;
    }

    void trajectoryResultCallback(const rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::WrappedResult& trajectory_result)
    {
        playing = false;

        RCLCPP_WARN(this->get_logger(), "i got in trajectory_result_callback");

        if(trajectory_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_WARN(this->get_logger(), "Successfully reached goal!");
        }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Trajectory goal was not reached.");
        }
    }

    void startRecordingServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        if(recording)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being recorded.");
            return;
        }

        if(playing)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot start recording, trajectory is currently being played.");
            return;
        }

        recording = true;

        publishStatus("TEACHING");

        auto enableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
        enableMappingClient->async_send_request(enableMappingRequest);
        return;
    }

    norlab_controllers_msgs::msg::PathSequence smoothTrajectoryLowPass(const norlab_controllers_msgs::msg::PathSequence& roughTrajectory)
    {
        norlab_controllers_msgs::msg::PathSequence smoothTrajectory(roughTrajectory);

        for(int i = 0; i < roughTrajectory.paths.size(); ++i)
        {
            for(int j = lowPassFilterWindowSize; j < roughTrajectory.paths[i].poses.size() - lowPassFilterWindowSize; ++j)
            {
                double windowNeighborSumX = 0;
                double windowNeighborSumY = 0;
                double windowNeighborSumZ = 0;
                for(int k = j - lowPassFilterWindowSize; k < j + lowPassFilterWindowSize + 1; ++k)
                {
                    windowNeighborSumX += roughTrajectory.paths[i].poses[k].pose.position.x;
                    windowNeighborSumY += roughTrajectory.paths[i].poses[k].pose.position.y;
                    windowNeighborSumZ += roughTrajectory.paths[i].poses[k].pose.position.z;
                }
                smoothTrajectory.paths[i].poses[j].pose.position.x = windowNeighborSumX / ((2 * lowPassFilterWindowSize) + 1);
                smoothTrajectory.paths[i].poses[j].pose.position.y = windowNeighborSumY / ((2 * lowPassFilterWindowSize) + 1);
                smoothTrajectory.paths[i].poses[j].pose.position.z = windowNeighborSumZ / ((2 * lowPassFilterWindowSize) + 1);
            }
        }
        for(int i = 0; i < lowPassFilterWindowSize; ++i)
        {
            smoothTrajectory.paths.front().poses.erase(smoothTrajectory.paths.front().poses.begin());
            smoothTrajectory.paths.back().poses.erase(smoothTrajectory.paths.back().poses.end() - 1);
        }

        return smoothTrajectory;
    }

    void stopRecordingServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        if(!recording)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already not being recorded.");
            return;
        }

        recording = false;

        publishStatus("IDLE: Stopped Recording");

        return;
    }

    void clearTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        plannedTrajectory.paths.clear();

        publishStatus("IDLE: Cleared Trajectory");

        return;
    }

    void smoothTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        plannedTrajectory = smoothTrajectoryLowPass(plannedTrajectory);
        RCLCPP_INFO(this->get_logger(), "Trajectory has been smoothened");
        return;
    }

    void cancelTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        if(!playing)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel trajectory, no trajectory is being played.");
            return;
        }

        playing = false;
        publishStatus("IDLE: Canceled Trajectory");

        // TODO: validate action call here
        followPathClient->async_cancel_all_goals();
        return;
    }

    double computeEuclideanDistanceBetweenPoses(const geometry_msgs::msg::Pose& firstPose, const geometry_msgs::msg::Pose& secondPose)
    {
        return sqrt(pow(firstPose.position.x - secondPose.position.x, 2) +
                    pow(firstPose.position.y - secondPose.position.y, 2) +
                    pow(firstPose.position.z - secondPose.position.z, 2));
    }

    double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
    {
        return std::atan2(2.0f * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z);
    }

    double computeTrajectoryYaw(const norlab_controllers_msgs::msg::PathSequence& trajectory, const geometry_msgs::msg::Pose& pose)
    {
        double dx = 0;
        double dy = 0;
        int pathIndex = 0;
        while(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 1.0 && pathIndex < trajectory.paths.size())
        {
            int poseIndex = 0;
            while(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 1.0 && poseIndex < trajectory.paths[pathIndex].poses.size())
            {
                dx = trajectory.paths[pathIndex].poses[poseIndex].pose.position.x - pose.position.x;
                dy = trajectory.paths[pathIndex].poses[poseIndex].pose.position.y - pose.position.y;
                ++poseIndex;
            }
            ++pathIndex;
        }
        return std::atan2(dy, dx);
    }

    void saveLTRServiceCallback(const std::shared_ptr<wiln::srv::SaveMapTraj::Request> req, std::shared_ptr<wiln::srv::SaveMapTraj::Response> res)
    {
        //TODO: force save to same working repository as mapper
        using namespace std::chrono_literals;
        auto saveMapRequest = std::make_shared<norlab_icp_mapper_ros::srv::SaveMap::Request>();
        std::string mapNameStem = req->file_name.data.substr(0, req->file_name.data.rfind('.'));
        std::string mapName = mapNameStem + ".vtk";
        saveMapRequest->map_file_name.data = mapName;
//        auto saveMapFuture = saveMapClient->async_send_request(saveMapRequest);
//        std::shared_ptr<norlab_icp_mapper_ros::srv::SaveMap::Request> request = std::make_shared<norlab_icp_mapper_ros::srv::SaveMap::Request>();
        RCLCPP_INFO(this->get_logger(), "calling /save_map");
        norlab_icp_mapper_ros::srv::SaveMap::Response response = rclcpp::call_service<norlab_icp_mapper_ros::srv::SaveMap>("save_map", saveMapRequest);
        RCLCPP_INFO(this->get_logger(), "/save_map done");

        std::rename(mapName.c_str(), req->file_name.data.c_str());
        std::ofstream ltrFile(req->file_name.data, std::ios::app);

        ltrFile << TRAJECTORY_DELIMITER << std::endl;
        ltrFile << "frame_id : " << plannedTrajectory.paths.front().poses.front().header.frame_id << std::endl;

        for(int i = 0; i < plannedTrajectory.paths.size(); ++i)
        {
            for(int j = 0; j < plannedTrajectory.paths[i].poses.size(); ++j)
            {
                ltrFile << plannedTrajectory.paths[i].poses[j].pose.position.x << ","
                        << plannedTrajectory.paths[i].poses[j].pose.position.y << ","
                        << plannedTrajectory.paths[i].poses[j].pose.position.z << ","
                        << plannedTrajectory.paths[i].poses[j].pose.orientation.x << ","
                        << plannedTrajectory.paths[i].poses[j].pose.orientation.y << ","
                        << plannedTrajectory.paths[i].poses[j].pose.orientation.z << ","
                        << plannedTrajectory.paths[i].poses[j].pose.orientation.w << std::endl;
            }
            if(i != plannedTrajectory.paths.size() - 1)
            {
                ltrFile << "changing direction" << std::endl;
            }
        }

        ltrFile.close();
        RCLCPP_INFO(this->get_logger(), "LTR file succesfully saved");
    }

    void loadLTR(std::string fileName, bool fromEnd)
    {
        using namespace std::chrono_literals;
        plannedTrajectory.paths.clear();
        std::ofstream mapFile("/tmp/map.vtk");
        std::ifstream ltrFile(fileName);
        std::string line;
        std::string pathFrameId;
        bool parsingMap = true;
        while(std::getline(ltrFile, line))
        {
            if(parsingMap)
            {
                if(line.find(TRAJECTORY_DELIMITER) != std::string::npos)
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
                if(plannedTrajectory.paths.empty())
                {
                    norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                    directionalPath.header.frame_id = pathFrameId;
                    directionalPath.header.stamp = this->now();
                    directionalPath.forward = true;
                    plannedTrajectory.paths.push_back(directionalPath);
                }

                if(line.find("changing direction") != std::string::npos)
                {
                    norlab_controllers_msgs::msg::DirectionalPath directionalPath;
                    directionalPath.header.frame_id = pathFrameId;
                    directionalPath.header.stamp = this->now();
                    directionalPath.forward = !plannedTrajectory.paths.back().forward;
                    plannedTrajectory.paths.push_back(directionalPath);
                    std::getline(ltrFile, line);
                }

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = pathFrameId;
                pose.header.stamp = this->now();
                int cursorPosition = line.find(",");
                pose.pose.position.x = std::stod(line.substr(0, cursorPosition));
                int previousCursorPosition = cursorPosition + 1;
                cursorPosition = line.find(",", previousCursorPosition);
                pose.pose.position.y = std::stod(line.substr(previousCursorPosition, cursorPosition));
                previousCursorPosition = cursorPosition + 1;
                cursorPosition = line.find(",", previousCursorPosition);
                pose.pose.position.z = std::stod(line.substr(previousCursorPosition, cursorPosition));
                previousCursorPosition = cursorPosition + 1;
                cursorPosition = line.find(",", previousCursorPosition);
                pose.pose.orientation.x = std::stod(line.substr(previousCursorPosition, cursorPosition));
                previousCursorPosition = cursorPosition + 1;
                cursorPosition = line.find(",", previousCursorPosition);
                pose.pose.orientation.y = std::stod(line.substr(previousCursorPosition, cursorPosition));
                previousCursorPosition = cursorPosition + 1;
                cursorPosition = line.find(",", previousCursorPosition);
                pose.pose.orientation.z = std::stod(line.substr(previousCursorPosition, cursorPosition));
                previousCursorPosition = cursorPosition + 1;
                cursorPosition = line.find("\n", previousCursorPosition);
                pose.pose.orientation.w = std::stod(line.substr(previousCursorPosition, cursorPosition));
                pose.header.frame_id = pathFrameId;
                plannedTrajectory.paths.back().poses.push_back(pose);
            }
        }
        ltrFile.close();
        mapFile.close();

        auto loadMapRequest = std::make_shared<norlab_icp_mapper_ros::srv::LoadMap::Request>();
        loadMapRequest->map_file_name.data = "/tmp/map.vtk";

        int pathIndex;
        int poseIndex;
        if(!fromEnd)
        {
            pathIndex = 0;
            poseIndex = 0;
        }
        else
        {
            pathIndex = plannedTrajectory.paths.size() - 1;
            poseIndex = plannedTrajectory.paths[pathIndex].poses.size() - 1;
        }

        loadMapRequest->pose.position.x = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.position.x;
        loadMapRequest->pose.position.y = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.position.y;
        loadMapRequest->pose.position.z = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.position.z;
        loadMapRequest->pose.orientation.x = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.x;
        loadMapRequest->pose.orientation.y = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.y;
        loadMapRequest->pose.orientation.z = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.z;
        loadMapRequest->pose.orientation.w = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.w;
        loadMapClient->async_send_request(loadMapRequest);

//        auto loadMapFuture = loadMapClient->async_send_request(loadMapRequest);
        RCLCPP_INFO(this->get_logger(), "calling /load_map");
        norlab_icp_mapper_ros::srv::LoadMap::Response response = rclcpp::call_service<norlab_icp_mapper_ros::srv::LoadMap>("load_map", loadMapRequest);
        RCLCPP_INFO(this->get_logger(), "/load_map done");

        std::remove("/tmp/map.vtk");

        publishPlannedTrajectory();
    }

    void publishPlannedTrajectory()
    {
        auto plannedPath = getNavPathFromPathSequence(plannedTrajectory);
        plannedTrajectoryPublisher->publish(plannedPath);
    }

    void publishRealTrajectory()
    {
        auto realPath = getNavPathFromPathSequence(realTrajectory);
        realTrajectoryPublisher->publish(realPath);
    }

    /**
     * @brief Publish WILN status
     *
     * @param status A String that contains the status
     */
    void publishStatus(const std::string& status)
    {
        auto statusMsg = std_msgs::msg::String();
        statusMsg.data = status;
        statusPublisher->publish(statusMsg);
    }

    void loadLTRServiceCallback(const std::shared_ptr<wiln::srv::LoadMapTraj::Request> req, std::shared_ptr<wiln::srv::LoadMapTraj::Response> res)
    {
        loadLTR(req->file_name.data, false);
        publishStatus("IDLE: Loaded LTR");
        return;
    }

    void loadLTRFromEndServiceCallback(const std::shared_ptr<wiln::srv::LoadMapTraj::Request> req, std::shared_ptr<wiln::srv::LoadMapTraj::Response> res)
    {
        loadLTR(req->file_name.data, true);
        publishStatus("IDLE: Loaded LTR From End");
        return;
    }

    void playLineTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        if(playing)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being played.");
            return;
        }

        if(recording)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play trajectory while recording.");
            return;
        }

        if(plannedTrajectory.paths.empty() || plannedTrajectory.paths.front().poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play an empty trajectory.");
            return;
        }

        robotPoseLock.lock();

        publishStatus("REPEATING");

        norlab_controllers_msgs::msg::PathSequence trajectory = plannedTrajectory;

        double robotPoseToTrajectoryStartDistance = computeEuclideanDistanceBetweenPoses(robotPose, trajectory.paths.front().poses.front().pose);
        double robotPoseToTrajectoryEndDistance = computeEuclideanDistanceBetweenPoses(robotPose, trajectory.paths.back().poses.back().pose);

        RCLCPP_INFO_STREAM(this->get_logger(), "distance to start "<<robotPoseToTrajectoryStartDistance);
        RCLCPP_INFO_STREAM(this->get_logger(), "distance to end "<<robotPoseToTrajectoryEndDistance);

        if(robotPoseToTrajectoryEndDistance < robotPoseToTrajectoryStartDistance)
        {
            tf2::Quaternion robotOrientation;

            std::reverse(trajectory.paths.begin(), trajectory.paths.end());
            for(int i = 0; i < trajectory.paths.size(); ++i)
            {
                std::reverse(trajectory.paths[i].poses.begin(), trajectory.paths[i].poses.end());
                for(int j = 0; j < trajectory.paths[i].poses.size(); ++j)
                {
                    tf2::fromMsg(trajectory.paths[i].poses[j].pose.orientation, robotOrientation);
                    trajectory.paths[i].poses[j].pose.orientation = tf2::toMsg((HALF_TURN_ROTATION * robotOrientation).normalized());
                }
            }
        }

        double robotPoseYaw = extractYawFromQuaternion(robotPose.orientation);
        double trajectoryStartYaw = computeTrajectoryYaw(trajectory, robotPose);
        double angleDistance = std::fabs(trajectoryStartYaw - robotPoseYaw);
        if(angleDistance > M_PI)
        {
            angleDistance = (2 * M_PI) - angleDistance;
        }

        if(angleDistance > M_PI_2)
        {
            tf2::Quaternion robotOrientation;

            for(int i = 0; i < trajectory.paths.size(); ++i)
            {
                trajectory.paths[i].forward = !plannedTrajectory.paths[i].forward;
                for(int j = 0; j < trajectory.paths[i].poses.size(); ++j)
                {
                    tf2::fromMsg(trajectory.paths[i].poses[j].pose.orientation, robotOrientation);
                    trajectory.paths[i].poses[j].pose.orientation = tf2::toMsg((HALF_TURN_ROTATION * robotOrientation).normalized());
                }
            }
        }
        robotPoseLock.unlock();

        playing = true;

        realTrajectory.paths.clear();

        auto disableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
        disableMappingClient->async_send_request(disableMappingRequest);

        // TODO: validate action call
        auto goal_msg = norlab_controllers_msgs::action::FollowPath::Goal();
        goal_msg.follower_options.init_mode.data = 1; // init_mode = 1 : continue
        goal_msg.follower_options.velocity.data = trajectorySpeed;
        goal_msg.path = trajectory;
        goal_msg.path.header.frame_id = trajectory.paths.front().poses.front().header.frame_id;
        goal_msg.path.header.stamp = this->now();

        auto send_goal_options = rclcpp_action::Client<norlab_controllers_msgs::action::FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&WilnNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
                std::bind(&WilnNode::trajectoryFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
                std::bind(&WilnNode::trajectoryResultCallback, this, std::placeholders::_1);
        followPathClient->async_send_goal(goal_msg, send_goal_options);

        return;
    }

    void playLoopTrajectoryServiceCallback(const std::shared_ptr<wiln::srv::PlayLoop::Request> req, std::shared_ptr<wiln::srv::PlayLoop::Response> res)
    {
        if(playing)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being played.");
            return;
        }

        if(recording)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play trajectory while recording.");
            return;
        }

        if(plannedTrajectory.paths.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play an empty trajectory.");
            return;
        }

        norlab_controllers_msgs::msg::PathSequence cutLoopTrajectory = plannedTrajectory;
        int poseIndex = cutLoopTrajectory.paths.back().poses.size() - 1;
        while(poseIndex >= 1 && computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.back().poses[poseIndex - 1].pose, cutLoopTrajectory.paths.front().poses.front().pose) <
                                computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.back().poses[poseIndex].pose, cutLoopTrajectory.paths.front().poses.front().pose))
        {
            cutLoopTrajectory.paths.back().poses.erase(cutLoopTrajectory.paths.back().poses.begin() + poseIndex);
            --poseIndex;
        }
        norlab_controllers_msgs::msg::PathSequence firstLoopTrajectory = cutLoopTrajectory;
        while(firstLoopTrajectory.paths.front().poses.size() > 0 &&
              computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.back().poses.back().pose, firstLoopTrajectory.paths.back().poses.back().pose) < 1.0)
        {
            firstLoopTrajectory.paths.front().poses.erase(firstLoopTrajectory.paths.back().poses.end() - 1);
        }
        norlab_controllers_msgs::msg::PathSequence lastLoopTrajectory = cutLoopTrajectory;
        while(lastLoopTrajectory.paths.front().poses.size() > 0 &&
              computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.front().poses.front().pose, lastLoopTrajectory.paths.front().poses.front().pose) < 1.0)
        {
            lastLoopTrajectory.paths.front().poses.erase(lastLoopTrajectory.paths.front().poses.begin());
        }
        norlab_controllers_msgs::msg::PathSequence middleLoopTrajectory = firstLoopTrajectory;
        while(middleLoopTrajectory.paths.front().poses.size() > 0 &&
              computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.front().poses.front().pose, middleLoopTrajectory.paths.front().poses.front().pose) < 1.0)
        {
            middleLoopTrajectory.paths.front().poses.erase(middleLoopTrajectory.paths.front().poses.begin());
        }

        playing = true;
        publishStatus("REPEATING: Loop");

        realTrajectory.paths.clear();

        auto disableMappingRequest = std::make_shared<std_srvs::srv::Empty::Request>();
        disableMappingClient->async_send_request(disableMappingRequest);

        // TODO: validate action call
        auto goal_msg = norlab_controllers_msgs::action::FollowPath::Goal();
        goal_msg.follower_options.init_mode.data = 1; // init_mode = 1 : continue
        goal_msg.follower_options.velocity.data = trajectorySpeed;
        goal_msg.path.header.frame_id = plannedTrajectory.paths.front().poses.front().header.frame_id;
        goal_msg.path.header.stamp = this->now();
        for(int i = 0; i < firstLoopTrajectory.paths.size(); ++i)
        {
            if(i != 0 && firstLoopTrajectory.paths[i].forward == goal_msg.path.paths.back().forward)
            {
                for(int j = 0; j < firstLoopTrajectory.paths[i].poses.size(); ++j)
                {
                    goal_msg.path.paths.back().poses.push_back(firstLoopTrajectory.paths[i].poses[j]);
                }
            }
            else
            {
                goal_msg.path.paths.push_back(firstLoopTrajectory.paths[i]);
            }
        }
        for(int i = 1; i < req->nb_loops.data - 1; ++i)
        {
            for(int j = 0; j < middleLoopTrajectory.paths.size(); ++j)
            {
                if(middleLoopTrajectory.paths[j].forward == goal_msg.path.paths.back().forward)
                {
                    for(int k = 0; k < middleLoopTrajectory.paths[j].poses.size(); ++k)
                    {
                        goal_msg.path.paths.back().poses.push_back(middleLoopTrajectory.paths[j].poses[k]);
                    }
                }
                else
                {
                    goal_msg.path.paths.push_back(middleLoopTrajectory.paths[j]);
                }
            }
        }

        for(int i = 0; i < lastLoopTrajectory.paths.size(); ++i)
        {
            if(lastLoopTrajectory.paths[i].forward == goal_msg.path.paths.back().forward)
            {
                for(int j = 0; j < lastLoopTrajectory.paths[i].poses.size(); ++j)
                {
                    goal_msg.path.paths.back().poses.push_back(lastLoopTrajectory.paths[i].poses[j]);
                }
            }
            else
            {
                goal_msg.path.paths.push_back(lastLoopTrajectory.paths[i]);
            }
        }

        auto send_goal_options = rclcpp_action::Client<norlab_controllers_msgs::action::FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&WilnNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
                std::bind(&WilnNode::trajectoryFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
                std::bind(&WilnNode::trajectoryResultCallback, this, std::placeholders::_1);
        followPathClient->async_send_goal(goal_msg);
        return;
    }

    nav_msgs::msg::Path getNavPathFromPathSequence(const norlab_controllers_msgs::msg::PathSequence& pathSequence)
    {
        nav_msgs::msg::Path navPath;
        navPath.header = pathSequence.header;
        for(const auto& path: pathSequence.paths)
        {
            for(const geometry_msgs::msg::PoseStamped& poseStamped: path.poses)
            {
                navPath.poses.push_back(poseStamped);
            }
        }
        return navPath;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<WilnNode>());
    rclcpp::executors::MultiThreadedExecutor executor;
    auto wiln_node = std::make_shared<WilnNode>();
    executor.add_node(wiln_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
