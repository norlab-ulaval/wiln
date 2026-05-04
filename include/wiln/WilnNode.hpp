#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "wiln/TeachRecorder.hpp"
#include "wiln/TrajectoryStreamer.hpp"
#include "wiln/PathDeformer.hpp"
#include "wiln/srv/save_map_traj.hpp"
#include "wiln/srv/load_map_traj.hpp"
#include "wiln/srv/play_loop.hpp"

#include <norlab_controllers_msgs/action/follow_path.hpp>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>

namespace wiln {

class WilnNode : public rclcpp::Node {
public:
    using FollowPath = norlab_controllers_msgs::action::FollowPath;

    WilnNode();

private:
    // Core Components
    std::unique_ptr<TeachRecorder> recorder_;
    std::unique_ptr<TrajectoryStreamer> streamer_;
    std::unique_ptr<PathDeformer> deformer_;

    // ROS 2 Interfaces
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_srv_;
    rclcpp::Service<wiln::srv::SaveMapTraj>::SharedPtr save_ltr_srv_;
    rclcpp::Service<wiln::srv::LoadMapTraj>::SharedPtr load_ltr_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr play_line_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancel_srv_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;

    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    
    // Internal State
    bool playing_ = false;
    bool driving_forward_ = true;
    geometry_msgs::msg::Pose current_pose_;
    std::mutex pose_mutex_;
    std::vector<Eigen::Vector3d> latest_obstacles_;
    std::mutex obs_mutex_;

    // Timer for streaming
    rclcpp::TimerBase::SharedPtr stream_timer_;

    // Callbacks
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    
    void streamLoop();

    // Service Handlers
    void handleStartRecording(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void handleStopRecording(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void handlePlayLine(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void handleSaveLTR(const std::shared_ptr<wiln::srv::SaveMapTraj::Request> req, std::shared_ptr<wiln::srv::SaveMapTraj::Response> res);
    void handleLoadLTR(const std::shared_ptr<wiln::srv::LoadMapTraj::Request> req, std::shared_ptr<wiln::srv::LoadMapTraj::Response> res);
};

} // namespace wiln
