#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_msgs/DirectionalPath.h>
#include <path_msgs/FollowPathActionResult.h>
#include <mutex>
#include <path_msgs/FollowPathAction.h>
#include <actionlib/client/simple_action_client.h>

std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> client;
std::atomic_bool recording(false);
std::atomic_bool playing(false);
path_msgs::DirectionalPath trajectory;
std::mutex trajectoryMutex;

void poseCallback(const geometry_msgs::PoseStamped& poseStamped)
{
	if(recording)
	{
		trajectoryMutex.lock();
		trajectory.poses.push_back(poseStamped);
		trajectoryMutex.unlock();
	}
}

void trajectoryResultCallback(const path_msgs::FollowPathActionResult& trajectoryResult)
{
	std::cout << trajectoryResult << std::endl;
	
	playing = false;
	
	// if result != path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS, ROS_WARN?
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
	trajectoryMutex.lock();
	trajectory.poses.clear();
	trajectoryMutex.unlock();
	
	return true;
}

bool playTrajectoryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(playing)
	{
		ROS_WARN("Trajectory is already being played.");
		return false;
	}
	
	recording = false;
	playing = true;
	
	path_msgs::FollowPathActionGoal goal;
	trajectoryMutex.lock();
	goal.goal.path.paths.push_back(trajectory);
	trajectoryMutex.unlock();
	goal.goal.follower_options.velocity = 1.0;
	goal.goal.follower_options.init_mode = path_msgs::FollowerOptions::INIT_MODE_CONTINUE;
	
	client->sendGoal(goal.goal);
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "norlab_teach_repeat_node");
	ros::NodeHandle nodeHandle;
	
	client = std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>(
			new actionlib::SimpleActionClient<path_msgs::FollowPathAction>("follow_path", true));
	client->waitForServer();
	
	ros::Subscriber poseSubscriber = nodeHandle.subscribe("pose", 1000, poseCallback);
	ros::Subscriber trajectoryResultSubscriber = nodeHandle.subscribe("follow_path/result", 1000, trajectoryResultCallback);
	
	ros::ServiceServer startRecordingService = nodeHandle.advertiseService("start_recording", startRecordingCallback);
	ros::ServiceServer stopRecordingService = nodeHandle.advertiseService("stop_recording", stopRecordingCallback);
	ros::ServiceServer clearTrajectoryService = nodeHandle.advertiseService("clear_trajectory", clearTrajectoryCallback);
	ros::ServiceServer playTrajectoryService = nodeHandle.advertiseService("play_trajectory", playTrajectoryCallback);
	
	ros::spin();
	
	return 0;
}
