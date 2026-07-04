#pragma once
/**
 * WilnRouteNode -- Route file I/O node.
 *
 * Responsibilities:
 *   - Save a trajectory to .ltr file (+ request VTK map save from ICP mapper).
 *   - Load a .ltr file and publish the trajectory (+ request VTK map load).
 *   - Publish the loaded/saved trajectory on /wiln/trajectory.
 *
 * Commands consumed from /wiln/command:
 *   "save:<filepath>"  -- Save current cached trajectory to file.
 *   "load:<filepath>"  -- Load trajectory from file and publish.
 *
 * External service calls (async):
 *   /mapping/save_map  -- Request ICP mapper to save the VTK point cloud.
 *   /mapping/load_map  -- Load the VTK map before publishing the trajectory.
 */

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>
#include <std_msgs/msg/string.hpp>

#include "wiln/TrajectoryManager.hpp"
#include "wiln/msg/wiln_state.hpp"

namespace wiln {

class WilnRouteNode : public rclcpp::Node {
public:
    WilnRouteNode();

private:
    using SaveMap = norlab_icp_mapper_ros::srv::SaveMap;
    using LoadMap = norlab_icp_mapper_ros::srv::LoadMap;

    // ----- Cached trajectory (from /wiln/trajectory sub) -----
    norlab_controllers_msgs::msg::PathSequence cached_trajectory_;
    std::mutex                                  traj_mutex_;
    bool                                        traj_received_{false};

    // ----- ROS interfaces -----
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                    command_sub_;
    rclcpp::Subscription<norlab_controllers_msgs::msg::PathSequence>::SharedPtr traj_sub_;

    rclcpp::Publisher<norlab_controllers_msgs::msg::PathSequence>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                        global_plan_pub_;
    rclcpp::Publisher<wiln::msg::WilnState>::SharedPtr                       state_pub_;

    // ----- Mapper service clients (async, non-blocking) -----
    rclcpp::Client<SaveMap>::SharedPtr save_map_client_;
    rclcpp::Client<LoadMap>::SharedPtr load_map_client_;

    // ----- Params -----
    std::string save_map_service_;
    std::string load_map_service_;
    bool create_parent_dirs_{true};
    double max_route_step_m_{1.0};
    double max_route_yaw_step_rad_{0.8};
    double max_route_z_span_m_{2.0};

    // ----- Callbacks -----
    void onCommand(std_msgs::msg::String::SharedPtr msg);
    void onTrajectory(norlab_controllers_msgs::msg::PathSequence::SharedPtr msg);

    // ----- Handlers -----
    void handleSave(const std::string& filepath);
    void handleLoad(const std::string& filepath);

    // ----- Helpers -----
    void publishTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj);
    void finishLoadedTrajectory(
        const norlab_controllers_msgs::msg::PathSequence& traj,
        size_t pose_count,
        const std::string& filepath);
    void publishState(uint8_t state_code, const std::string& detail, const std::string& route_name = "");
    bool trajectoryUsable(
        const norlab_controllers_msgs::msg::PathSequence& traj,
        size_t* pose_count = nullptr,
        std::string* reason = nullptr) const;
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
    static double wrapToPi(double angle);
};

} // namespace wiln
