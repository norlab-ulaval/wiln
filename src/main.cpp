#include <ros/ros.h>
#include <fstream>
#include <mutex>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_msgs/DirectionalPath.h>
#include <path_msgs/FollowPathGoal.h>
#include <path_msgs/FollowPathAction.h>
#include <path_msgs/FollowPathActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include "wiln/SaveMapTraj.h"
#include "wiln/LoadMapTraj.h"
#include <norlab_icp_mapper_ros//SaveMap.h>
#include <norlab_icp_mapper_ros//LoadMap.h>
#include "nav_msgs/Odometry.h"

float delayBetweenWaypoints;
float trajectorySpeed;
int lowPassFilterWindowSize;

bool playing;
bool recording;
ros::Publisher plannedTrajectoryPublisher;
ros::Publisher realTrajectoryPublisher;
std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> simpleActionClient;
path_msgs::DirectionalPath plannedTrajectory;
path_msgs::DirectionalPath realTrajectory;
geometry_msgs::Pose robotPose;
std::mutex robotPoseLock;
ros::ServiceClient saveMapClient;
ros::ServiceClient loadMapClient;
ros::ServiceClient enableMappingClient;
ros::ServiceClient disableMappingClient;

const int FRAME_ID_START_POSITION = 11; // length of string : "frame_id : "
const std::string TRAJECTORY_DELIMITER = "#############################";

//class SaveMap;

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

void icpOdomCallback(const nav_msgs::Odometry& odometry)
{
    robotPoseLock.lock();
    robotPose = odometry.pose.pose;
    robotPoseLock.unlock();
}

void publishTrajectory(const ros::Publisher& publisher, path_msgs::DirectionalPath& trajectory, const std::string& frame_id, const ros::Time& stamp)
{
	trajectory.header.frame_id = frame_id;
	trajectory.header.stamp = stamp;
	trajectory.forward = true;
	
	path_msgs::PathSequence pathSequence;
	pathSequence.header.frame_id = frame_id;
	pathSequence.header.stamp = stamp;
	pathSequence.paths.push_back(trajectory);
	
	publisher.publish(pathSequence);
}

void plannedTrajectoryCallback(const geometry_msgs::PoseStamped& poseStamped)
{
    if(recording)
    {
	    double distance = 0;
        if (!plannedTrajectory.poses.empty())
        {
            geometry_msgs::PoseStamped lastPose = plannedTrajectory.poses[plannedTrajectory.poses.size()-1];
            distance = std::sqrt(std::pow(poseStamped.pose.position.x - lastPose.pose.position.x, 2) +
                std::pow(poseStamped.pose.position.y - lastPose.pose.position.y, 2) +
                std::pow(poseStamped.pose.position.z - lastPose.pose.position.z, 2));
        }
        if (distance >= 0.05 || plannedTrajectory.poses.empty())
        {
            plannedTrajectory.poses.push_back(poseStamped);
            publishTrajectory(plannedTrajectoryPublisher, plannedTrajectory, poseStamped.header.frame_id, ros::Time::now());
            plannedTrajectory.header.frame_id = poseStamped.header.frame_id;
            plannedTrajectory.header.stamp = poseStamped.header.stamp;
        }
    }
}

void realTrajectoryCallback(const geometry_msgs::PoseStamped& poseStamped)
{
	if(playing)
    {
        double distance = 0;
        if (!realTrajectory.poses.empty())
        {
            geometry_msgs::PoseStamped lastPose = realTrajectory.poses[realTrajectory.poses.size()-1];
            distance = std::sqrt(std::pow(poseStamped.pose.position.x - lastPose.pose.position.x, 2) +
                                 std::pow(poseStamped.pose.position.y - lastPose.pose.position.y, 2) +
                                 std::pow(poseStamped.pose.position.z - lastPose.pose.position.z, 2));
        }
        if (distance >= 0.05 || realTrajectory.poses.empty())
        {
            realTrajectory.poses.push_back(poseStamped);
            publishTrajectory(realTrajectoryPublisher, realTrajectory, poseStamped.header.frame_id, ros::Time::now());
        }
    }
}

void trajectoryResultCallback(const path_msgs::FollowPathActionResult& trajectoryResult)
{
	playing = false;
	
	if(trajectoryResult.result.status == path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS)
	{
		ROS_INFO("Successfully reached goal!");
	}
	else
	{
		ROS_WARN_STREAM("Trajectory goal was not reached, got status " + FOLLOW_PATH_RESULTS.at(trajectoryResult.result.status) + ".");
	}
}

bool startRecordingServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(recording)
	{
		ROS_WARN("Trajectory is already being recorded.");
		return false;
	}
	
	if(playing)
	{
		ROS_WARN("Cannot start recording, trajectory is currently being played.");
		return false;
	}
	
	recording = true;

    std_srvs::Empty enableMappingService = std_srvs::Empty();
    enableMappingClient.call(enableMappingService);

	return true;
}

path_msgs::DirectionalPath smoothTrajectoryLowPass(const path_msgs::DirectionalPath& roughTrajectory)
{
    path_msgs::DirectionalPath smoothTrajectory(roughTrajectory);
    double windowNeighborSumX;
    double windowNeighborSumY;
    double windowNeighborSumZ;
    double windowTotalDistance;
    double windowTotalDistanceInverse;

    for (int i = lowPassFilterWindowSize; i < roughTrajectory.poses.size() - lowPassFilterWindowSize; i++)
    {
        windowNeighborSumX = 0;
        windowNeighborSumY = 0;
        windowNeighborSumZ = 0;
        windowTotalDistance = 0;
        for (int j = i - lowPassFilterWindowSize; j < i + lowPassFilterWindowSize + 1; i++)
        {
            windowNeighborSumX += roughTrajectory.poses[j].pose.position.x;
            windowNeighborSumY += roughTrajectory.poses[j].pose.position.y;
            windowNeighborSumZ += roughTrajectory.poses[j].pose.position.z;
            windowTotalDistance += 1;
        }
        windowTotalDistanceInverse = 1/windowTotalDistance;
        smoothTrajectory.poses[i].pose.position.x = windowTotalDistanceInverse * windowNeighborSumX;
        smoothTrajectory.poses[i].pose.position.y = windowTotalDistanceInverse * windowNeighborSumY;
        smoothTrajectory.poses[i].pose.position.z = windowTotalDistanceInverse * windowNeighborSumZ;
    }
    return smoothTrajectory;
}

bool stopRecordingServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(!recording)
	{
		ROS_WARN("Trajectory is already not being recorded.");
		return false;
	}

	recording = false;
    plannedTrajectory = smoothTrajectoryLowPass(plannedTrajectory);
	return true;
}

bool clearTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	plannedTrajectory.poses.clear();
	return true;
}

double computeEuclideanDistanceBetweenPoses(const geometry_msgs::Pose& firstPose, const geometry_msgs::Pose& secondPose)
{
    return sqrt(pow(firstPose.position.x - secondPose.position.x, 2) +
        pow(firstPose.position.y - secondPose.position.y, 2) +
        pow(firstPose.position.z - secondPose.position.z, 2));
}

double extractYawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
    return std::atan2(2.0f * (quaternion.w * quaternion.z + quaternion.x *
        quaternion.y), quaternion.w * quaternion.w + quaternion.x *
        quaternion.x - quaternion.y * quaternion.y - quaternion.z *
        quaternion.z);
}

double computeTrajectoryYaw(const path_msgs::DirectionalPath& trajectory, const geometry_msgs::Pose& pose)
{
    int indexTrajectory = 0;
    double dx = 0;
    double dy = 0;
    while(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 1.0 && indexTrajectory < trajectory.poses.size())
    {
        dx = trajectory.poses[indexTrajectory].pose.position.x - pose.position.x;
        dy = trajectory.poses[indexTrajectory].pose.position.y - pose.position.y;
        indexTrajectory++;
    }
    return std::atan2(dy, dx);
}

bool playTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
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

    if(plannedTrajectory.poses.empty())
    {
        ROS_WARN("Cannot play an empty trajectory.");
        return false;
    }

    robotPoseLock.lock();

    double robotPoseToTrajectoryStartDistance = computeEuclideanDistanceBetweenPoses(robotPose, plannedTrajectory.poses.front().pose);
    double robotPoseToTrajectoryEndDistance = computeEuclideanDistanceBetweenPoses(robotPose, plannedTrajectory.poses.back().pose);
    if(robotPoseToTrajectoryEndDistance < robotPoseToTrajectoryStartDistance)
    {
        std::reverse(plannedTrajectory.poses.begin(), plannedTrajectory.poses.end());
        ROS_INFO("Reversed trajectory");
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
        plannedTrajectory.forward = false;
        ROS_INFO("Going backwards");
    }
    else
    {
        plannedTrajectory.forward = true;
        ROS_INFO("Going forwards");
    }
    robotPoseLock.unlock();

    playing = true;

    realTrajectory.poses.clear();

    std_srvs::Empty disableMappingService = std_srvs::Empty();
    disableMappingClient.call(disableMappingService);

    path_msgs::FollowPathGoal goal;
    goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
    goal.follower_options.velocity = trajectorySpeed;
    goal.path.header.frame_id = plannedTrajectory.poses[0].header.frame_id;
    goal.path.header.stamp = ros::Time::now();;
    goal.path.paths.push_back(plannedTrajectory);
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
    ltrFile << "frame_id : " << plannedTrajectory.poses[0].header.frame_id << std::endl;

    for(int i=0; i<plannedTrajectory.poses.size(); i++)
    {
        ltrFile << plannedTrajectory.poses[i].pose.position.x << ","
                << plannedTrajectory.poses[i].pose.position.y << ","
                << plannedTrajectory.poses[i].pose.position.z << ","
                << plannedTrajectory.poses[i].pose.orientation.x << ","
                << plannedTrajectory.poses[i].pose.orientation.y << ","
                << plannedTrajectory.poses[i].pose.orientation.z << ","
                << plannedTrajectory.poses[i].pose.orientation.w << std::endl;
    }

    ltrFile.close();
    return true;
}

void loadLTR(std::string fileName, bool fromEnd)
{

    std::ofstream mapFile("/tmp/map.vtk");
    std::ifstream ltrFile(fileName);
    path_msgs::DirectionalPath loadTrajectory;
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
            plannedTrajectory.poses.push_back(pose);
        }
    }
    ltrFile.close();
    mapFile.close();

    norlab_icp_mapper_ros::LoadMap loadMapService;
    loadMapService.request.map_file_name.data = "/tmp/map.vtk";

    int positionIndex;
    if(!fromEnd)
    {
        positionIndex = 0;
    }
    else
    {
        positionIndex = plannedTrajectory.poses.size() - 1;
    }

    loadMapService.request.pose.position.x = plannedTrajectory.poses[positionIndex].pose.position.x;
    loadMapService.request.pose.position.y = plannedTrajectory.poses[positionIndex].pose.position.y;
    loadMapService.request.pose.position.z = plannedTrajectory.poses[positionIndex].pose.position.z;
    loadMapService.request.pose.orientation.x = plannedTrajectory.poses[positionIndex].pose.orientation.x;
    loadMapService.request.pose.orientation.y = plannedTrajectory.poses[positionIndex].pose.orientation.y;
    loadMapService.request.pose.orientation.z = plannedTrajectory.poses[positionIndex].pose.orientation.z;
    loadMapService.request.pose.orientation.w = plannedTrajectory.poses[positionIndex].pose.orientation.w;
    loadMapClient.call(loadMapService);

    std::remove("/tmp/map.vtk");

    plannedTrajectory.header.frame_id = pathFrameId;
    for (int i = 0; i < plannedTrajectory.poses.size(); i++)
    {
        plannedTrajectory.poses[i].header.frame_id = pathFrameId;
    }

    publishTrajectory(plannedTrajectoryPublisher, plannedTrajectory, pathFrameId, ros::Time::now());
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
	
	ros::Subscriber plannedTrajectorySubscriber = nodeHandle.subscribe("pose_in", 1000, plannedTrajectoryCallback);
	ros::Subscriber realTrajectorySubscriber = nodeHandle.subscribe("pose_in", 1000, realTrajectoryCallback);
	ros::Subscriber trajectoryResultSubscriber = nodeHandle.subscribe("follow_path/result", 1000, trajectoryResultCallback);
    ros::Subscriber icpOdomSubscriber = nodeHandle.subscribe("icp_odom", 1000, icpOdomCallback);
	
	plannedTrajectoryPublisher = nodeHandle.advertise<path_msgs::PathSequence>("planned_trajectory", 1000, true);
	realTrajectoryPublisher = nodeHandle.advertise<path_msgs::PathSequence>("real_trajectory", 1000, true);
	
	ros::ServiceServer startRecordingService = nodeHandle.advertiseService("start_recording", startRecordingServiceCallback);
	ros::ServiceServer stopRecordingService = nodeHandle.advertiseService("stop_recording", stopRecordingServiceCallback);
    ros::ServiceServer playTrajectoryService = nodeHandle.advertiseService("play_trajectory", playTrajectoryServiceCallback);
    ros::ServiceServer clearTrajectoryService = nodeHandle.advertiseService("clear_trajectory", clearTrajectoryServiceCallback);
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
