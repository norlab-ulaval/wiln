#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_msgs/DirectionalPath.h>
#include <path_msgs/FollowPathGoal.h>
#include <path_msgs/FollowPathAction.h>
#include <path_msgs/FollowPathActionResult.h>
#include <actionlib/client/simple_action_client.h>

ros::Publisher plannedTrajectoryPublisher;
ros::Publisher realTrajectoryPublisher;
float trajectorySpeed;
bool recording;
bool playing;
std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> actionlibClient;
path_msgs::DirectionalPath plannedTrajectory;
path_msgs::DirectionalPath realTrajectory;

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
		plannedTrajectory.poses.push_back(poseStamped);
		publishTrajectory(plannedTrajectoryPublisher, plannedTrajectory, poseStamped.header.frame_id, ros::Time::now());
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
	actionlibClient->sendGoal(goal);
	
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
	
	plannedTrajectoryPublisher = nodeHandle.advertise<path_msgs::PathSequence>("planned_trajectory", 1000);
	realTrajectoryPublisher = nodeHandle.advertise<path_msgs::PathSequence>("real_trajectory", 1000);
	
	ros::ServiceServer startRecordingService = nodeHandle.advertiseService("start_recording", startRecordingServiceCallback);
	ros::ServiceServer stopRecordingService = nodeHandle.advertiseService("stop_recording", stopRecordingServiceCallback);
	ros::ServiceServer clearTrajectoryService = nodeHandle.advertiseService("clear_trajectory", clearTrajectoryServiceCallback);
	ros::ServiceServer playTrajectoryService = nodeHandle.advertiseService("play_trajectory", playTrajectoryServiceCallback);
	
	privateNodeHandle.param<float>("trajectory_speed", trajectorySpeed, 5.0);
	
	recording = false;
	playing = false;
	
	actionlibClient = std::unique_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>(
			new actionlib::SimpleActionClient<path_msgs::FollowPathAction>("follow_path", true));
	
	ROS_INFO("Waiting for server...");
	actionlibClient->waitForServer();
	ROS_INFO("Connected to server");
	
	ros::spin();
	
	return 0;
}
