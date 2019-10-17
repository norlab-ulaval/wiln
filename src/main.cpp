#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_msgs/DirectionalPath.h>
#include <path_msgs/FollowPathActionResult.h>
#include <mutex>
#include <path_msgs/FollowPathAction.h>
#include <actionlib/client/simple_action_client.h>

float trajectorySpeed;
ros::Publisher pathPublisher;
std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> client;
std::atomic_bool recording(false);
std::atomic_bool playing(false);
path_msgs::DirectionalPath trajectory;

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

void poseCallback(const geometry_msgs::PoseStamped& poseStamped)
{
	if(recording)
	{
		trajectory.poses.push_back(poseStamped);
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

bool startRecordingCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(playing)
	{
		ROS_WARN("Cannot start recording, trajectory is currently being played.");
		return false;
	}
	
	recording = true;
	return true;
}

bool stopRecordingCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	recording = false;
	return true;
}

bool clearTrajectoryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	trajectory.poses.clear();
	return true;
}

bool playTrajectoryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(playing)
	{
		ROS_WARN("Trajectory is already being played.");
		return false;
	}
	
	if(trajectory.poses.empty())
	{
		ROS_WARN("Cannot play empty trajectory.");
		return false;
	}
	
	recording = false;
	playing = true;
	
	std::string frame_id = trajectory.poses[0].header.frame_id;
	ros::Time stamp = ros::Time::now();
	trajectory.header.frame_id = frame_id;
	trajectory.header.stamp = stamp;
	
	path_msgs::FollowPathActionGoal goal;
	goal.goal.path.paths.push_back(trajectory);
	goal.goal.path.header.frame_id = frame_id;
	goal.goal.path.header.stamp = stamp;
	goal.goal.follower_options.velocity = trajectorySpeed;
	goal.goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
	client->sendGoal(goal.goal);
	
	path_msgs::PathSequence pathSequence;
	pathSequence.paths.push_back(trajectory);
	pathSequence.header.frame_id = frame_id;
	pathSequence.header.stamp = stamp;
	pathPublisher.publish(pathSequence);
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "norlab_teach_repeat_node");
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privateNodeHandle("~");
	
	privateNodeHandle.param<float>("trajectory_speed", trajectorySpeed, 5.0);
	
	ros::Subscriber poseSubscriber = nodeHandle.subscribe("pose_in", 1000, poseCallback);
	ros::Subscriber trajectoryResultSubscriber = nodeHandle.subscribe("follow_path/result", 1000, trajectoryResultCallback);
	
	pathPublisher = nodeHandle.advertise<path_msgs::PathSequence>("path", 1000);
	
	ros::ServiceServer startRecordingService = nodeHandle.advertiseService("start_recording", startRecordingCallback);
	ros::ServiceServer stopRecordingService = nodeHandle.advertiseService("stop_recording", stopRecordingCallback);
	ros::ServiceServer clearTrajectoryService = nodeHandle.advertiseService("clear_trajectory", clearTrajectoryCallback);
	ros::ServiceServer playTrajectoryService = nodeHandle.advertiseService("play_trajectory", playTrajectoryCallback);
	
	client = std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>(
			new actionlib::SimpleActionClient<path_msgs::FollowPathAction>("follow_path", true));
	trajectory.forward = true;
	
	ROS_INFO("Waiting for server...");
	client->waitForServer();
	ROS_INFO("Connected to server");
	
	ros::spin();
	
	return 0;
}
