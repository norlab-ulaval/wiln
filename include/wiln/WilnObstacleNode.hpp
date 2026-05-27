#pragma once
/**
 * WilnObstacleNode -- Standalone obstacle aggregation node.
 *
 * Subscribes to N configurable LiDAR PointCloud2 topics, transforms them
 * into a common frame, crops/height-filters/voxel-downsamples around the
 * current robot pose, and publishes the result on /wiln/obstacles at a
 * configurable rate (default 10 Hz).
 *
 * Consumers (e.g. wiln_replay_node) subscribe to /wiln/obstacles and are
 * fully decoupled from this node -- no shared memory, no services.
 */

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "wiln/ObstacleManager.hpp"

namespace wiln {

class WilnObstacleNode : public rclcpp::Node {
public:
    WilnObstacleNode();

private:
    // TF
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Core obstacle processing
    std::unique_ptr<ObstacleManager> obstacle_manager_;
    std::string                      target_frame_;

    // Robot pose (updated from odom subscription)
    geometry_msgs::msg::Pose current_pose_;
    std::mutex               pose_mutex_;
    bool                     pose_received_{false};

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Callbacks
    void onOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void publishObstacles();

    // Helpers
    static sensor_msgs::msg::PointCloud2 toPointCloud2(
        const std::vector<Eigen::Vector3d>& points,
        const std::string&                  frame_id,
        const rclcpp::Time&                 stamp);
};

} // namespace wiln
