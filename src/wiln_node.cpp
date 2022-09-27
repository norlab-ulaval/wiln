#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <mutex>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <path_msgs/DirectionalPath.h>
//#include <path_msgs/FollowPathGoal.h>
//#include <path_msgs/FollowPathAction.h>
//#include <path_msgs/FollowPathActionResult.h>
#include <wiln/msg/directional_path.hpp>
#include <wiln/msg/path_sequence.hpp>
// TODO: Find equivalent ROS 2 path messages
#include <nav_msgs/msg/path.hpp>
//#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wiln/srv/save_map_traj.hpp>
#include <wiln/srv/load_map_traj.hpp>
#include <wiln/srv/play_loop.hpp>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class WilnNode : public rclcpp::Node
{
public:
    WilnNode() :
        Node("wiln_node")
    {
        odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("odom_in", 1000,
                                                                              std::bind(&WilnNode::odomCallback, this,
                                                                                        std::placeholders::_1));
        commandedVelocitySubscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_in", 1000,
                                                                              std::bind(&WilnNode::commandVelocityCallback, this,
                                                                                        std::placeholders::_1));

        plannedTrajectoryPublisher = this->create_publisher<wiln::msg::PathSequence>("planned_trajectory", 1000);
        realTrajectoryPublisher = this->create_publisher<wiln::msg::PathSequence>("real_trajectory", 1000);
    }

private:
    bool playing;
    bool recording;
    std::atomic_bool drivingForward;
//    ros::Publisher plannedTrajectoryPublisher;
//    ros::Publisher realTrajectoryPublisher;
//    std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> simpleActionClient;
    wiln::msg::PathSequence plannedTrajectory;
    wiln::msg::PathSequence realTrajectory;
    geometry_msgs::msg::Pose robotPose;
    std::mutex robotPoseLock;
    rclcpp::Service<wiln::srv::SaveMapTraj>::SharedPtr saveMapClient;
    rclcpp::Service<wiln::srv::LoadMapTraj>::SharedPtr loadMapClient;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableMappingClient;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableMappingClient;
    std::atomic_bool lastDrivingDirection;

    const int FRAME_ID_START_POSITION = 11; // length of string : "frame_id : "

    const std::string TRAJECTORY_DELIMITER = "#############################";
    float delayBetweenWaypoints;

    float trajectorySpeed;
    int lowPassFilterWindowSize;
    const std::map<int8_t, std::string> FOLLOW_PATH_RESULTS = {
            { 0u, "RESULT_STATUS_STOPPED_BY_SUPERVISOR" },
            { 1u, "RESULT_STATUS_UNKNOWN" },
            { 2u, "RESULT_STATUS_OBSTACLE" },
            { 3u, "RESULT_STATUS_SUCCESS" },
            { 4u, "RESULT_STATUS_ABORTED" },
            { 5u, "RESULT_STATUS_INTERNAL_ERROR" },
            { 6u, "RESULT_STATUS_SLAM_FAIL" },
            { 7u, "RESULT_STATUS_TF_FAIL" },
            { 8u, "RESULT_STATUS_PATH_LOST" },
            { 9u, "RESULT_STATUS_TIMEOUT" },
    };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commandedVelocitySubscription;

    rclcpp::Publisher<wiln::msg::PathSequence>::SharedPtr plannedTrajectoryPublisher;
    rclcpp::Publisher<wiln::msg::PathSequence>::SharedPtr realTrajectoryPublisher;

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

    void plannedTrajectoryCallback(const geometry_msgs::msg::PoseStamped& poseStamped)
    {
        if(recording)
        {
            if(plannedTrajectory.paths.empty())
            {
                wiln::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
//                directionalPath.header.stamp = ros::Time::now();
                directionalPath.header.stamp = this->now();
                directionalPath.forward = true;
                plannedTrajectory.paths.push_back(directionalPath);
            }

            double distance = 0;
            if(!plannedTrajectory.paths.back().poses.empty())
            {
                geometry_msgs::msg::PoseStamped lastPose = plannedTrajectory.paths.back().poses.back();
                distance = std::sqrt(std::pow(poseStamped.pose.position.x - lastPose.pose.position.x, 2) +
                                     std::pow(poseStamped.pose.position.y - lastPose.pose.position.y, 2) +
                                     std::pow(poseStamped.pose.position.z - lastPose.pose.position.z, 2));
            }

            if (distance >= 0.05 || plannedTrajectory.paths.back().poses.empty())
            {
                if(lastDrivingDirection.load() != drivingForward.load())
                {
                    wiln::msg::DirectionalPath directionalPath;
                    directionalPath.header.frame_id = poseStamped.header.frame_id;
                    directionalPath.header.stamp = this->now();
                    directionalPath.forward = drivingForward.load();
                    plannedTrajectory.paths.push_back(directionalPath);
                }

                plannedTrajectory.paths.back().poses.push_back(poseStamped);
                plannedTrajectoryPublisher->publish(plannedTrajectory);
                plannedTrajectory.header.frame_id = poseStamped.header.frame_id;
                plannedTrajectory.header.stamp = poseStamped.header.stamp;
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
                wiln::msg::DirectionalPath directionalPath;
                directionalPath.header.frame_id = poseStamped.header.frame_id;
                directionalPath.header.stamp = this->now();
                directionalPath.forward = true;
                realTrajectory.paths.push_back(directionalPath);
            }

            double distance = 0;
            if(!realTrajectory.paths.back().poses.empty())
            {
                geometry_msgs::msg::PoseStamped lastPose = realTrajectory.paths.back().poses.back();
                distance = std::sqrt(std::pow(poseStamped.pose.position.x - lastPose.pose.position.x, 2) +
                                     std::pow(poseStamped.pose.position.y - lastPose.pose.position.y, 2) +
                                     std::pow(poseStamped.pose.position.z - lastPose.pose.position.z, 2));
            }

            if (distance >= 0.05 || realTrajectory.paths.back().poses.empty())
            {
                if(lastDrivingDirection.load() != drivingForward.load())
                {
                    wiln::msg::DirectionalPath directionalPath;
                    directionalPath.header.frame_id = poseStamped.header.frame_id;
                    directionalPath.header.stamp = this->now();
                    directionalPath.forward = drivingForward.load();
                    realTrajectory.paths.push_back(directionalPath);
                }

                realTrajectory.paths.back().poses.push_back(poseStamped);
                realTrajectoryPublisher->publish(realTrajectory);
            }
            lastDrivingDirection.store(drivingForward.load());
        }
    }

    // TODO: Re-program actionlib client here
//    void trajectoryResultCallback(const path_msgs::FollowPathActionResult& trajectoryResult)
//    {
//        playing = false;
//
//        if(trajectoryResult.result.status == path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS)
//        {
//            ROS_INFO("Successfully reached goal!");
//        }
//        else
//        {
//            ROS_WARN_STREAM("Trajectory goal was not reached, got status " + FOLLOW_PATH_RESULTS.at(trajectoryResult.result.status) + ".");
//        }
//    }
//
//    bool startRecordingServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
//    {
//        if(recording)
//        {
//            ROS_WARN("Trajectory is already being recorded.");
//            return false;
//        }
//
//        if(playing)
//        {
//            ROS_WARN("Cannot start recording, trajectory is currently being played.");
//            return false;
//        }
//
//        recording = true;
//
//        std_srvs::Empty enableMappingService = std_srvs::Empty();
//        enableMappingClient.call(enableMappingService);
//
//        return true;
//    }
    wiln::msg::PathSequence smoothTrajectoryLowPass(const wiln::msg::PathSequence& roughTrajectory)
    {
        wiln::msg::PathSequence smoothTrajectory(roughTrajectory);

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

    bool stopRecordingServiceCallback(std_srvs::srv::Empty::Request& req, std_srvs::srv::Empty::Response& res)
    {
        if(!recording)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already not being recorded.");
            return false;
        }

        recording = false;
        return true;
    }

    bool clearTrajectoryServiceCallback(std_srvs::srv::Empty::Request& req, std_srvs::srv::Empty::Response& res)
    {
        plannedTrajectory.paths.clear();
        return true;
    }

    bool smoothTrajectoryServiceCallback(std_srvs::srv::Empty::Request& req, std_srvs::srv::Empty::Response& res)
    {
        plannedTrajectory = smoothTrajectoryLowPass(plannedTrajectory);
        return true;
    }

    double computeEuclideanDistanceBetweenPoses(const geometry_msgs::msg::Pose& firstPose, const geometry_msgs::msg::Pose& secondPose)
    {
        return sqrt(pow(firstPose.position.x - secondPose.position.x, 2) +
                    pow(firstPose.position.y - secondPose.position.y, 2) +
                    pow(firstPose.position.z - secondPose.position.z, 2));
    }

    double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
    {
        return std::atan2(2.0f * (quaternion.w *
                                    quaternion.z +
                                    quaternion.x *
                                    quaternion.y),
                                    quaternion.w *
                                    quaternion.w +
                                    quaternion.x *
                                    quaternion.x -
                                    quaternion.y *
                                    quaternion.y -
                                    quaternion.z *
                                                                                                                                                          quaternion.z);
    }

    double computeTrajectoryYaw(const wiln::msg::PathSequence& trajectory, const geometry_msgs::msg::Pose& pose)
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

    bool playTrajectoryServiceCallback(std_srvs::srv::Empty::Request& req, std_srvs::srv::Empty::Response& res)
    {
        if(playing)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already being played.");
            return false;
        }

        if(recording)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play trajectory while recording.");
            return false;
        }

        if(plannedTrajectory.paths.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cannot play an empty trajectory.");
            return false;
        }

        robotPoseLock.lock();

        double robotPoseToTrajectoryStartDistance = computeEuclideanDistanceBetweenPoses(robotPose, plannedTrajectory.paths.front().poses.front().pose);
        double robotPoseToTrajectoryEndDistance = computeEuclideanDistanceBetweenPoses(robotPose, plannedTrajectory.paths.back().poses.back().pose);
        if(robotPoseToTrajectoryEndDistance < robotPoseToTrajectoryStartDistance)
        {

            std::reverse(plannedTrajectory.paths.begin(), plannedTrajectory.paths.end());
            for(int i = 0; i < plannedTrajectory.paths.size(); ++i)
            {
                std::reverse(plannedTrajectory.paths[i].poses.begin(), plannedTrajectory.paths[i].poses.end());
            }
        }

        double robotPoseYaw = extractYawFromQuaternion(robotPose.orientation);
        double trajectoryStartYaw = computeTrajectoryYaw(plannedTrajectory, robotPose);
        double angleDistance = std::fabs(trajectoryStartYaw - robotPoseYaw);
        if (angleDistance > M_PI)
        {
            angleDistance = (2 * M_PI) - angleDistance;
        }

        if(angleDistance > M_PI_2)
        {
            for(int i = 0; i < plannedTrajectory.paths.size(); ++i)
            {
                plannedTrajectory.paths[i].forward = !plannedTrajectory.paths[i].forward;
            }
        }
        robotPoseLock.unlock();

        playing = true;

        realTrajectory.paths.clear();

        std_srvs::srv::Empty disableMappingService = std_srvs::srv::Empty();
//        disableMappingClient.call(disableMappingService);
//        disableMappingClient->;
        // TODO: Validate conversion above

        wiln::msg::FollowPathGoal goal;
        goal.follower_options.init_mode = wiln::msg::FollowerOptions::INIT_MODE_CONTINUE;
        goal.follower_options.velocity = trajectorySpeed;
        goal.path.header.frame_id = plannedTrajectory.paths.front().poses.front().header.frame_id;
        goal.path.header.stamp = this->now();
        for(int i = 0; i < plannedTrajectory.paths.size(); ++i)
        {
            goal.path.paths.push_back(plannedTrajectory.paths[i]);
        }
        simpleActionClient->sendGoal(goal);

        return true;
    }

};



bool playLoopTrajectoryServiceCallback(wiln::PlayLoop::Request& req, wiln::PlayLoop::Response& res)
{
	if(playing)
	{
		ROS_WARN("Trajectory is already being played.");
		return false;
	}

	if(recording)
	{
		ROS_WARN("Cannot play trajectory while recording.");
		return false;
	}

	if(plannedTrajectory.paths.empty())
	{
		ROS_WARN("Cannot play an empty trajectory.");
		return false;
	}

	path_msgs::PathSequence cutLoopTrajectory = plannedTrajectory;
	int poseIndex = cutLoopTrajectory.paths.back().poses.size() - 1;
	while(poseIndex >= 1 && computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.back().poses[poseIndex - 1].pose, cutLoopTrajectory.paths.front().poses.front().pose) <
							computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.back().poses[poseIndex].pose, cutLoopTrajectory.paths.front().poses.front().pose))
	{
		cutLoopTrajectory.paths.back().poses.erase(cutLoopTrajectory.paths.back().poses.begin() + poseIndex);
		--poseIndex;
	}
	path_msgs::PathSequence firstLoopTrajectory = cutLoopTrajectory;
	while(firstLoopTrajectory.paths.front().poses.size() > 0 && computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.back().poses.back().pose, firstLoopTrajectory.paths.back().poses.back().pose) < 1.0)
	{
		firstLoopTrajectory.paths.front().poses.erase(firstLoopTrajectory.paths.back().poses.end()-1);
	}
	path_msgs::PathSequence lastLoopTrajectory = cutLoopTrajectory;
	while(lastLoopTrajectory.paths.front().poses.size() > 0 && computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.front().poses.front().pose, lastLoopTrajectory.paths.front().poses.front().pose) < 1.0)
	{
		lastLoopTrajectory.paths.front().poses.erase(lastLoopTrajectory.paths.front().poses.begin());
	}
	path_msgs::PathSequence middleLoopTrajectory = firstLoopTrajectory;
	while(middleLoopTrajectory.paths.front().poses.size() > 0 && computeEuclideanDistanceBetweenPoses(cutLoopTrajectory.paths.front().poses.front().pose, middleLoopTrajectory.paths.front().poses.front().pose) < 1.0)
	{
		middleLoopTrajectory.paths.front().poses.erase(middleLoopTrajectory.paths.front().poses.begin());
		
	}

	playing = true;

	realTrajectory.paths.clear();

	std_srvs::Empty disableMappingService = std_srvs::Empty();
	disableMappingClient.call(disableMappingService);

	path_msgs::FollowPathGoal goal;
	goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
	goal.follower_options.velocity = trajectorySpeed;
	goal.path.header.frame_id = plannedTrajectory.paths.front().poses.front().header.frame_id;
	goal.path.header.stamp = ros::Time::now();
	for(int i = 0; i < firstLoopTrajectory.paths.size(); ++i)
	{
		if(i != 0 && firstLoopTrajectory.paths[i].forward == goal.path.paths.back().forward)
		{
			for(int j = 0; j < firstLoopTrajectory.paths[i].poses.size(); ++j)
			{
				goal.path.paths.back().poses.push_back(firstLoopTrajectory.paths[i].poses[j]);
			}
		}
		else
		{
			goal.path.paths.push_back(firstLoopTrajectory.paths[i]);
		}
	}
	for(int i = 1; i < req.nbLoops-1; ++i)
	{
		for(int j = 0; j < middleLoopTrajectory.paths.size(); ++j)
		{
			if(middleLoopTrajectory.paths[j].forward == goal.path.paths.back().forward)
			{
				for(int k = 0; k < middleLoopTrajectory.paths[j].poses.size(); ++k)
				{
					goal.path.paths.back().poses.push_back(middleLoopTrajectory.paths[j].poses[k]);
				}
			}
			else
			{
				goal.path.paths.push_back(middleLoopTrajectory.paths[j]);
			}
		}
	}

	for(int i = 0; i < lastLoopTrajectory.paths.size(); ++i)
	{
		if(lastLoopTrajectory.paths[i].forward == goal.path.paths.back().forward)
		{
			for(int j = 0; j < lastLoopTrajectory.paths[i].poses.size(); ++j)
			{
				goal.path.paths.back().poses.push_back(lastLoopTrajectory.paths[i].poses[j]);
			}
		}
		else
		{
			goal.path.paths.push_back(lastLoopTrajectory.paths[i]);
		}
	}
	simpleActionClient->sendGoal(goal);

	return true;
}

bool cancelTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(!playing)
	{
		ROS_WARN("Cannot cancel trajectory, no trajectory is being played.");
		return false;
	}

	playing = false;

	simpleActionClient->cancelAllGoals();

	return true;
}

bool saveLTRServiceCallback(wiln::SaveMapTraj::Request& req, wiln::SaveMapTraj::Response& res)
{
    norlab_icp_mapper_ros::SaveMap saveMapService;
    std::string mapName = req.file_name.substr(0, req.file_name.rfind('.')) + ".vtk";
    saveMapService.request.map_file_name.data = mapName;
    saveMapClient.call(saveMapService);
    std::rename(mapName.c_str(), req.file_name.c_str());
    std::ofstream ltrFile(req.file_name, std::ios::app);

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
    return true;
}

void loadLTR(std::string fileName, bool fromEnd)
{
    std::ofstream mapFile("/tmp/map.vtk");
    std::ifstream ltrFile(fileName);
    std::string line;
    std::string pathFrameId;
    bool parsingMap = true;
    while (std::getline(ltrFile, line))
    {
        if(parsingMap)
        {
            if (line.find(TRAJECTORY_DELIMITER) != std::string::npos)
            {
                std::getline(ltrFile, line);
                pathFrameId = line.substr(FRAME_ID_START_POSITION);
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
				path_msgs::DirectionalPath directionalPath;
				directionalPath.header.frame_id = pathFrameId;
				directionalPath.header.stamp = ros::Time::now();
				directionalPath.forward = true;
				plannedTrajectory.paths.push_back(directionalPath);
			}

			if(line.find("changing direction") != std::string::npos)
			{
				path_msgs::DirectionalPath directionalPath;
				directionalPath.header.frame_id = pathFrameId;
				directionalPath.header.stamp = ros::Time::now();
				directionalPath.forward = !plannedTrajectory.paths.back().forward;
				plannedTrajectory.paths.push_back(directionalPath);
				std::getline(ltrFile, line);
			}

            geometry_msgs::PoseStamped pose;
            int cursorPosition = line.find(",");
            pose.pose.position.x = std::stod(line.substr(0, cursorPosition));
            int previousCursorPosition = cursorPosition + 1;
            cursorPosition = line.find("," , previousCursorPosition);
            pose.pose.position.y = std::stod(line.substr(previousCursorPosition, cursorPosition));
            previousCursorPosition = cursorPosition + 1;
            cursorPosition = line.find("," , previousCursorPosition);
            pose.pose.position.z = std::stod(line.substr(previousCursorPosition, cursorPosition));
            previousCursorPosition = cursorPosition + 1;
            cursorPosition = line.find("," , previousCursorPosition);
            pose.pose.orientation.x = std::stod(line.substr(previousCursorPosition, cursorPosition));
            previousCursorPosition = cursorPosition + 1;
            cursorPosition = line.find("," , previousCursorPosition);
            pose.pose.orientation.y = std::stod(line.substr(previousCursorPosition, cursorPosition));
            previousCursorPosition = cursorPosition + 1;
            cursorPosition = line.find("," , previousCursorPosition);
            pose.pose.orientation.z = std::stod(line.substr(previousCursorPosition, cursorPosition));
            previousCursorPosition = cursorPosition + 1;
            cursorPosition = line.find("\n" , previousCursorPosition);
            pose.pose.orientation.w = std::stod(line.substr(previousCursorPosition, cursorPosition));
			pose.header.frame_id = pathFrameId;
            plannedTrajectory.paths.back().poses.push_back(pose);
        }
    }
    ltrFile.close();
    mapFile.close();

    norlab_icp_mapper_ros::LoadMap loadMapService;
    loadMapService.request.map_file_name.data = "/tmp/map.vtk";

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

    loadMapService.request.pose.position.x = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.position.x;
    loadMapService.request.pose.position.y = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.position.y;
    loadMapService.request.pose.position.z = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.position.z;
    loadMapService.request.pose.orientation.x = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.x;
    loadMapService.request.pose.orientation.y = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.y;
    loadMapService.request.pose.orientation.z = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.z;
    loadMapService.request.pose.orientation.w = plannedTrajectory.paths[pathIndex].poses[poseIndex].pose.orientation.w;
    loadMapClient.call(loadMapService);

    std::remove("/tmp/map.vtk");

	plannedTrajectoryPublisher.publish(plannedTrajectory);
}

bool loadLTRServiceCallback(wiln::LoadMapTraj::Request& req, wiln::LoadMapTraj::Response& res)
{
    loadLTR(req.file_name, false);
    return true;
}

bool loadLTRFromEndServiceCallback(wiln::LoadMapTraj::Request& req, wiln::LoadMapTraj::Response& res)
{
    loadLTR(req.file_name, true);
    return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wiln_node");
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privateNodeHandle("~");

	drivingForward.store(true);
	lastDrivingDirection.store(true);

	ros::Subscriber plannedTrajectorySubscriber = nodeHandle.subscribe("pose_in", 1000, plannedTrajectoryCallback);
	ros::Subscriber realTrajectorySubscriber = nodeHandle.subscribe("pose_in", 1000, realTrajectoryCallback);
	ros::Subscriber trajectoryResultSubscriber = nodeHandle.subscribe("follow_path/result", 1000, trajectoryResultCallback);
    ros::Subscriber icpOdomSubscriber = nodeHandle.subscribe("icp_odom", 1000, icpOdomCallback);
    ros::Subscriber commandVelocitySubscriber = nodeHandle.subscribe("cmd_vel", 1000, commandVelocityCallback);

	plannedTrajectoryPublisher = nodeHandle.advertise<path_msgs::PathSequence>("planned_trajectory", 1000, true);
	realTrajectoryPublisher = nodeHandle.advertise<path_msgs::PathSequence>("real_trajectory", 1000, true);

	ros::ServiceServer startRecordingService = nodeHandle.advertiseService("start_recording", startRecordingServiceCallback);
	ros::ServiceServer stopRecordingService = nodeHandle.advertiseService("stop_recording", stopRecordingServiceCallback);
    ros::ServiceServer playTrajectoryService = nodeHandle.advertiseService("play_trajectory", playTrajectoryServiceCallback);
    ros::ServiceServer playLoopTrajectoryService = nodeHandle.advertiseService("play_loop_trajectory", playLoopTrajectoryServiceCallback);
    ros::ServiceServer clearTrajectoryService = nodeHandle.advertiseService("clear_trajectory", clearTrajectoryServiceCallback);
    ros::ServiceServer smoothTrajectoryService = nodeHandle.advertiseService("smooth_trajectory", smoothTrajectoryServiceCallback);
    ros::ServiceServer cancelTrajectoryService = nodeHandle.advertiseService("cancel_trajectory", cancelTrajectoryServiceCallback);

    ros::ServiceServer saveLTRService = nodeHandle.advertiseService("save_ltr", saveLTRServiceCallback);
    ros::ServiceServer loadLTRService = nodeHandle.advertiseService("load_ltr", loadLTRServiceCallback);
    ros::ServiceServer loadLTRFromEndService = nodeHandle.advertiseService("load_ltr_from_end", loadLTRFromEndServiceCallback);

    saveMapClient = nodeHandle.serviceClient<norlab_icp_mapper_ros::SaveMap>("save_map");
    loadMapClient = nodeHandle.serviceClient<norlab_icp_mapper_ros::LoadMap>("load_map");
    enableMappingClient = nodeHandle.serviceClient<std_srvs::Empty>("enable_mapping");
    disableMappingClient = nodeHandle.serviceClient<std_srvs::Empty>("disable_mapping");

	privateNodeHandle.param<float>("trajectory_speed", trajectorySpeed, 1.0);
	privateNodeHandle.param<float>("delay_between_waypoints", delayBetweenWaypoints, 0.5);
	privateNodeHandle.param<int>("low_pass_filter_window_size", lowPassFilterWindowSize, 5);

	recording = false;
	playing = false;

	simpleActionClient = std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>(
			new actionlib::SimpleActionClient<path_msgs::FollowPathAction>("follow_path", true));

	ros::MultiThreadedSpinner spinner;
    spinner.spin();

	return 0;
}
