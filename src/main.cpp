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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "norlab_teach_repeat/SaveTraj.h"
#include "norlab_teach_repeat/LoadTraj.h"
#include "norlab_teach_repeat/SaveMapTraj.h"
#include "norlab_teach_repeat/LoadMapTraj.h"
#include "map_msgs/SaveMap.h"
#include "nav_msgs/Odometry.h"

ros::Publisher plannedTrajectoryPublisher;
ros::Publisher realTrajectoryPublisher;
float delayBetweenWaypoints;
float trajectorySpeed;
bool recording;
bool playing;
std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> simpleActionClient;
path_msgs::DirectionalPath plannedTrajectory;
path_msgs::DirectionalPath realTrajectory;

geometry_msgs::Pose robotPose;
std::mutex robotPoseLock;

ros::ServiceClient client_saveMap;
ros::ServiceClient client_loadMap;
ros::ServiceClient client_enable_Mapping;
ros::ServiceClient client_disable_Mapping;

ros::Time lastTimeWaypointWasRecorded;

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
	}
    }
}

void realTrajectoryCallback(const geometry_msgs::PoseStamped& poseStamped)
{
	if(playing)
	{
		realTrajectory.poses.push_back(poseStamped);
		publishTrajectory(realTrajectoryPublisher, realTrajectory, poseStamped.header.frame_id, ros::Time::now());
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

    std_srvs::Empty srv_enableMapping = std_srvs::Empty();
    client_enable_Mapping.call(srv_enableMapping);

	return true;
}

bool stopRecordingServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(!recording)
	{
		ROS_WARN("Trajectory is already not being recorded.");
		return false;
	}
	
	recording = false;
	return true;
}

bool clearTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	plannedTrajectory.poses.clear();
	return true;
}

void reverseTrajectory()
{
    std::reverse(plannedTrajectory.poses.begin(), plannedTrajectory.poses.end());
//    for(int i=0; i<plannedTrajectory.poses.size()-1; i++) {
//        double angle = std::atan2(
//                plannedTrajectory.poses[i + 1].pose.position.y - plannedTrajectory.poses[i].pose.position.y,
//                plannedTrajectory.poses[i + 1].pose.position.x - plannedTrajectory.poses[i].pose.position.x);
//        tf2::Quaternion quaternion(angle, 0, 0);
//        plannedTrajectory.poses[i].pose.orientation = tf2::toMsg(quaternion);
//    }
}

void reverseRobotDirection()
{
    plannedTrajectory.forward = !plannedTrajectory.forward;
}

bool reverseTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    reverseTrajectory();
    return true;
}

bool reverseRobotDirServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    reverseRobotDirection();
    return true;
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
	
	playing = true;
	
	realTrajectory.poses.clear();

	std_srvs::Empty srv_disableMapping = std_srvs::Empty();
    client_disable_Mapping.call(srv_disableMapping);

	std::string frame_id = plannedTrajectory.poses[0].header.frame_id;
	ros::Time stamp = ros::Time::now();
	
	plannedTrajectory.header.frame_id = frame_id;
	plannedTrajectory.header.stamp = stamp;
	
	path_msgs::FollowPathGoal goal;
	goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
	goal.follower_options.velocity = trajectorySpeed;
	goal.path.header.frame_id = frame_id;
	goal.path.header.stamp = stamp;
	goal.path.paths.push_back(plannedTrajectory);
	simpleActionClient->sendGoal(goal);
	
	return true;
}

bool playReverseTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
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

    reverseTrajectory();
    reverseRobotDirection();

    playing = true;

    realTrajectory.poses.clear();

    std_srvs::Empty srv_disableMapping = std_srvs::Empty();
    client_disable_Mapping.call(srv_disableMapping);

    std::string frame_id = plannedTrajectory.poses[0].header.frame_id;
    ros::Time stamp = ros::Time::now();

    plannedTrajectory.header.frame_id = frame_id;
    plannedTrajectory.header.stamp = stamp;

    path_msgs::FollowPathGoal goal;
    goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
    goal.follower_options.velocity = trajectorySpeed;
    goal.path.header.frame_id = frame_id;
    goal.path.header.stamp = stamp;
    goal.path.paths.push_back(plannedTrajectory);
    simpleActionClient->sendGoal(goal);

    return true;
}

bool playAutoTrajectoryServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    double robotPoseToTrajStartDist;
    double robotPoseToTrajEndDist;
    double robotPoseToTrajStartAng;
    double siny_cosp;
    double cosy_cosp;
    double robotPoseYaw;
    double trajStartYaw;
    double angleDist;

    int lastTrajPose;
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
    lastTrajPose = plannedTrajectory.poses.size() - 1;
    // reverse traj if robot is closer to last traj pose
    robotPoseToTrajStartDist = pow(pow(robotPose.position.x - plannedTrajectory.poses[0].pose.position.x, 2) +
                                   pow(robotPose.position.y - plannedTrajectory.poses[0].pose.position.y, 2) +
                                   pow(robotPose.position.z - plannedTrajectory.poses[0].pose.position.z, 2), 0.5);
    robotPoseToTrajEndDist = pow(pow(robotPose.position.x - plannedTrajectory.poses[lastTrajPose].pose.position.x, 2) +
                                 pow(robotPose.position.y - plannedTrajectory.poses[lastTrajPose].pose.position.y, 2) +
                                 pow(robotPose.position.z - plannedTrajectory.poses[lastTrajPose].pose.position.z, 2), 0.5);
    if(robotPoseToTrajEndDist < robotPoseToTrajStartDist)
    {
        reverseTrajectory();
        ROS_ERROR("reversed trajectory");
    }

    // yaw (z-axis rotation)
    robotPoseYaw = std::atan2(2.0f * (robotPose.orientation.w * robotPose.orientation.z + robotPose.orientation.x *
            robotPose.orientation.y), robotPose.orientation.w * robotPose.orientation.w + robotPose.orientation.x *
            robotPose.orientation.x - robotPose.orientation.y * robotPose.orientation.y - robotPose.orientation.z *
            robotPose.orientation.z);
//    trajStartYaw = std::atan2(2.0f * (plannedTrajectory.poses[0].pose.orientation.w * plannedTrajectory.poses[0].pose.orientation.z +
//            plannedTrajectory.poses[0].pose.orientation.x * plannedTrajectory.poses[0].pose.orientation.y), plannedTrajectory.poses[0].pose.orientation.w *
//            plannedTrajectory.poses[0].pose.orientation.w + plannedTrajectory.poses[0].pose.orientation.x * plannedTrajectory.poses[0].pose.orientation.x -
//            plannedTrajectory.poses[0].pose.orientation.y * plannedTrajectory.poses[0].pose.orientation.y - plannedTrajectory.poses[0].pose.orientation.z *
//            plannedTrajectory.poses[0].pose.orientation.z);
    int indexTraj = 0;
    double distTraj = 0;
    double dx = 0;
    double dy = 0;
    while(distTraj < 1.0)
    {
        dx = plannedTrajectory.poses[indexTraj].pose.position.x - robotPose.position.x;
        dy = plannedTrajectory.poses[indexTraj].pose.position.y - robotPose.position.y;
        distTraj = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        indexTraj++;
    }
    trajStartYaw = std::atan2(dy, dx);
    ROS_ERROR_STREAM(robotPoseYaw);
    ROS_ERROR_STREAM(trajStartYaw);

    angleDist = std::fabs(trajStartYaw - robotPoseYaw);
    if (angleDist > M_PI)
        angleDist = (2 * M_PI) - angleDist;
    if(angleDist > M_PI_2)
    {
//        reverseRobotDirection();
        plannedTrajectory.forward = false;
        ROS_ERROR("Going backwards");
    }
    else
    {
        plannedTrajectory.forward = true;
        ROS_ERROR("Going forwards");
    }
    robotPoseLock.unlock();

    playing = true;

    realTrajectory.poses.clear();

    std_srvs::Empty srv_disableMapping = std_srvs::Empty();
    client_disable_Mapping.call(srv_disableMapping);

    std::string frame_id = plannedTrajectory.poses[0].header.frame_id;
    ros::Time stamp = ros::Time::now();

    plannedTrajectory.header.frame_id = frame_id;
    plannedTrajectory.header.stamp = stamp;

    path_msgs::FollowPathGoal goal;
    goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
    goal.follower_options.velocity = trajectorySpeed;
    goal.path.header.frame_id = frame_id;
    goal.path.header.stamp = stamp;
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

bool saveTrajectoryServiceFun(norlab_teach_repeat::SaveTraj::Request& req, norlab_teach_repeat::SaveTraj::Response& res)
{
    std::ofstream outFile(req.file_name);

    outFile << "frame_id : " << plannedTrajectory.header.frame_id << std::endl;

    for(int i=0; i<plannedTrajectory.poses.size(); i++)
    {
        outFile << plannedTrajectory.poses[i].pose.position.x << ","
                << plannedTrajectory.poses[i].pose.position.y << ","
                << plannedTrajectory.poses[i].pose.orientation.x << ","
                << plannedTrajectory.poses[i].pose.orientation.y << ","
                << plannedTrajectory.poses[i].pose.orientation.z << ","
                << plannedTrajectory.poses[i].pose.orientation.w << std::endl;
    }
    outFile.close();
    return true;
}

bool loadTrajectoryServiceFun(norlab_teach_repeat::LoadTraj::Request& req, norlab_teach_repeat::LoadTraj::Response& res)
{
    std::ifstream Trajectory(req.file_name);
    path_msgs::DirectionalPath loadTrajectory;
    bool trajSwitch;
    std::string load_frame_id;
    std::string line;
    while (std::getline(Trajectory, line))
    {
        if (trajSwitch)
        {
            geometry_msgs::PoseStamped pose;
            int pos = line.find(",");
            pose.pose.position.x = std::stod(line.substr(0, pos));
            int prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.position.y = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.orientation.x = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.orientation.y = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.orientation.z = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("\n" , prev_pos);
            pose.pose.orientation.w = std::stod(line.substr(prev_pos, pos));
            plannedTrajectory.poses.push_back(pose);
        }
        else if (line.find("frame_id : ") != std::string::npos) {
            int frame_start = 11; // length of string : "frame_id : "
            int frame_end = line.find("/n");
            load_frame_id = line.substr(frame_start, frame_end);
            trajSwitch = true;
        }
    }

    Trajectory.close();

    plannedTrajectory.header.frame_id = load_frame_id;
    for (int i = 0; i < plannedTrajectory.poses.size(); i++)
    {
        plannedTrajectory.poses[i].header.frame_id = load_frame_id;
    }

    // publish trajectory
    publishTrajectory(plannedTrajectoryPublisher, plannedTrajectory, load_frame_id, ros::Time::now());

    return true;
}

bool saveTrajectoryMapServiceFun(norlab_teach_repeat::SaveMapTraj::Request& req, norlab_teach_repeat::SaveMapTraj::Response& res)
{
    map_msgs::SaveMap srv;
    std::string mapName = req.file_name.substr(0, req.file_name.rfind('.')) + ".vtk";
    srv.request.filename.data = mapName;
    client_saveMap.call(srv);
    std::rename(mapName.c_str(), req.file_name.c_str());
    std::ofstream ltrFile(req.file_name, std::ios::app);

//    for (const auto &e : plannedTrajectory.poses) outFile << e << "\n";
    ltrFile << "#############################" << std::endl;
    ltrFile << "frame_id : " << plannedTrajectory.poses[0].header.frame_id << std::endl;

    for(int i=0; i<plannedTrajectory.poses.size(); i++)
    {
        ltrFile << plannedTrajectory.poses[i].pose.position.x << ","
                << plannedTrajectory.poses[i].pose.position.y << ","
                << plannedTrajectory.poses[i].pose.orientation.x << ","
                << plannedTrajectory.poses[i].pose.orientation.y << ","
                << plannedTrajectory.poses[i].pose.orientation.z << ","
                << plannedTrajectory.poses[i].pose.orientation.w << std::endl;
    }


    ltrFile.close();
    return true;
}

bool loadTrajectoryMapServiceFun(norlab_teach_repeat::LoadMapTraj::Request& req, norlab_teach_repeat::LoadMapTraj::Response& res)
{
    std::ofstream mapFile("/tmp/map.vtk");
    std::ifstream ltrFile(req.file_name);
    path_msgs::DirectionalPath loadTrajectory;
    std::string line;
    std::string load_frame_id;
    bool trajSwitch = false;
    while (std::getline(ltrFile, line))
    {
        if (trajSwitch)
        {
            geometry_msgs::PoseStamped pose;
            int pos = line.find(",");
            pose.pose.position.x = std::stod(line.substr(0, pos));
            int prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.position.y = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.orientation.x = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.orientation.y = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("," , prev_pos);
            pose.pose.orientation.z = std::stod(line.substr(prev_pos, pos));
            prev_pos = pos + 1;
            pos = line.find("\n" , prev_pos);
            pose.pose.orientation.w = std::stod(line.substr(prev_pos, pos));
            plannedTrajectory.poses.push_back(pose);
        }
	else if (line.find("#############################") != std::string::npos) {
	        std::getline(ltrFile, line);
	       	int frame_start = 11; // length of string : "frame_id : "
	        int frame_end = line.find("/n", frame_start);
	        load_frame_id = line.substr(frame_start);
            trajSwitch = true;
        }
	else
	{
	    mapFile << line << std::endl;
	}
    }
//    plannedTrajectory.poses
    ltrFile.close();
    mapFile.close();
   
    map_msgs::SaveMap srv;
    srv.request.filename.data = "/tmp/map.vtk";
    client_loadMap.call(srv);

    std::remove("/tmp/map.vtk");

    plannedTrajectory.header.frame_id = load_frame_id;
    for (int i = 0; i < plannedTrajectory.poses.size(); i++)
    {
        plannedTrajectory.poses[i].header.frame_id = load_frame_id;
    }

    // publish trajectory
    publishTrajectory(plannedTrajectoryPublisher, plannedTrajectory, load_frame_id, ros::Time::now());

    return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "norlab_teach_repeat_node");
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
	ros::ServiceServer clearTrajectoryService = nodeHandle.advertiseService("clear_trajectory", clearTrajectoryServiceCallback);
	ros::ServiceServer playTrajectoryService = nodeHandle.advertiseService("play_trajectory", playTrajectoryServiceCallback);
	ros::ServiceServer cancelTrajectoryService = nodeHandle.advertiseService("cancel_trajectory", cancelTrajectoryServiceCallback);
    ros::ServiceServer reverseTrajectoryService = nodeHandle.advertiseService("reverse_trajectory", reverseTrajectoryServiceCallback);
    ros::ServiceServer reverseRobotDirService = nodeHandle.advertiseService("reverse_dir", reverseRobotDirServiceCallback);
    ros::ServiceServer playReverseTrajectoryService = nodeHandle.advertiseService("play_reverse_trajectory", playReverseTrajectoryServiceCallback);
    ros::ServiceServer playAutoTrajectoryService = nodeHandle.advertiseService("play_auto", playAutoTrajectoryServiceCallback);
    ros::ServiceServer saveTrajectoryService = nodeHandle.advertiseService("save_trajectory", saveTrajectoryServiceFun);
    ros::ServiceServer loadTrajectoryService = nodeHandle.advertiseService("load_trajectory", loadTrajectoryServiceFun);
    ros::ServiceServer saveTrajectoryMapService = nodeHandle.advertiseService("save_map_trajectory", saveTrajectoryMapServiceFun);
    ros::ServiceServer loadTrajectoryMapService = nodeHandle.advertiseService("load_map_trajectory", loadTrajectoryMapServiceFun);

    client_saveMap = nodeHandle.serviceClient<map_msgs::SaveMap>("save_map");
    client_loadMap = nodeHandle.serviceClient<map_msgs::SaveMap>("load_map");

    client_enable_Mapping = nodeHandle.serviceClient<std_srvs::Empty>("enable_mapping");
    client_disable_Mapping = nodeHandle.serviceClient<std_srvs::Empty>("disable_mapping");

	privateNodeHandle.param<float>("trajectory_speed", trajectorySpeed, 5.0);
	
	privateNodeHandle.param<float>("delay_between_waypoints", delayBetweenWaypoints, 0.5);
	privateNodeHandle.param<float>("trajectory_speed", trajectorySpeed, 1.0);
	
	recording = false;
	playing = false;
	
	simpleActionClient = std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>(
			new actionlib::SimpleActionClient<path_msgs::FollowPathAction>("follow_path", true));
	
	ros::spin();
	
	return 0;
}
