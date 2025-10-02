#ifndef WILN_NODE_HPP
#define WILN_NODE_HPP

#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <norlab_custom_interfaces/srv/save_map_traj.hpp>
#include <norlab_custom_interfaces/srv/load_map_traj.hpp>
#include <norlab_custom_interfaces/srv/play_loop.hpp>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <norlab_controllers_msgs/action/follow_path.hpp>

enum State
{
    IDLE,
    RECORDING,
    PLAYING
};

class WilnNode : public rclcpp::Node
{
public:
    WilnNode();

private:
    State currentState;
    nav_msgs::msg::Path plannedTrajectory;
    nav_msgs::msg::Path realTrajectory;
    geometry_msgs::msg::PoseStamped robotPose;
    std::mutex robotPoseLock;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plannedTrajectoryPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr realTrajectoryPublisher;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr statePublisher;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startRecordingService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stopRecordingService;
    rclcpp::Service<norlab_custom_interfaces::srv::SaveMapTraj>::SharedPtr saveMapTrajService;
    rclcpp::Service<norlab_custom_interfaces::srv::LoadMapTraj>::SharedPtr loadMapTrajService;
    rclcpp::Service<norlab_custom_interfaces::srv::LoadMapTraj>::SharedPtr loadMapTrajFromEndService;
    rclcpp::Service<norlab_custom_interfaces::srv::PlayLoop>::SharedPtr playLoopService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr playLineService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancelTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr smoothTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverseTrajectoryService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr flipTrajectoryService;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr enableMappingClient;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr disableMappingClient;
    rclcpp::Client<norlab_icp_mapper_ros::srv::SaveMap>::SharedPtr saveMapClient;
    rclcpp::Client<norlab_icp_mapper_ros::srv::LoadMap>::SharedPtr loadMapClient;
    rclcpp_action::Client<norlab_controllers_msgs::action::FollowPath>::SharedPtr followPathClient;

    rclcpp::TimerBase::SharedPtr stateTimer;
    rclcpp::TimerBase::SharedPtr updateTimer;

    const int FRAME_ID_START_POSITION = 11;
    const std::string TRAJECTORY_DELIMITER = "#############################";
    std::string odomTopic;
    std::string followPathTopic;
    float distanceBetweenWaypoints;
    float angleBetweenWaypoints;
    float trajectorySpeed;
    int smoothingWindowSize;
    float loopClosureLinearTolerance;
    float loopClosureAngularTolerance;

    void initParameters();
    void updateParameters();
    void initSubscribers();
    void initPublishers();
    void initServices();
    void initClients();
    void initTimers();
    void odomCallback(const nav_msgs::msg::Odometry &odomIn);
    void updatePlannedTrajectory(const geometry_msgs::msg::PoseStamped &currentPose);
    void updateRealTrajectory(const geometry_msgs::msg::PoseStamped &currentPose);
    void startRecordingServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void stopRecordingServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void clearTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void reverseTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void flipTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void smoothTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void cancelTrajectoryServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void saveLTRServiceCallback(const std::shared_ptr<norlab_custom_interfaces::srv::SaveMapTraj::Request> req, std::shared_ptr<norlab_custom_interfaces::srv::SaveMapTraj::Response> res);
    bool saveLTR(std::string fileName);
    void loadLTRServiceCallback(const std::shared_ptr<norlab_custom_interfaces::srv::LoadMapTraj::Request> req, std::shared_ptr<norlab_custom_interfaces::srv::LoadMapTraj::Response> res);
    void loadLTRFromEndServiceCallback(const std::shared_ptr<norlab_custom_interfaces::srv::LoadMapTraj::Request> req, std::shared_ptr<norlab_custom_interfaces::srv::LoadMapTraj::Response> res);
    bool loadLTR(std::string fileName, bool fromEnd);
    void enableMapping();
    void disableMapping();
    void saveMap(std::string folderName);
    void loadMap(geometry_msgs::msg::Pose pose, std::string fileNameMap);
    void publishPlannedTrajectory();
    void publishRealTrajectory();
    void publishState();
    void playLineServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void playLine();
    void playLoopServiceCallback(const std::shared_ptr<norlab_custom_interfaces::srv::PlayLoop::Request> req, std::shared_ptr<norlab_custom_interfaces::srv::PlayLoop::Response> res);
    void playLoop(int nbLoops);
    void sendFollowPathAction(nav_msgs::msg::Path &path);
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::SharedPtr &trajectoryGoalHandle);
    void trajectoryFeedbackCallback(rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::SharedPtr, const std::shared_ptr<const norlab_controllers_msgs::action::FollowPath::Feedback> feedback);
    void trajectoryResultCallback(const rclcpp_action::ClientGoalHandle<norlab_controllers_msgs::action::FollowPath>::WrappedResult &trajectory_result);
};

#endif // WILN_NODE_HPP
