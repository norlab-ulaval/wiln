#pragma once
/**
 * WilnTeachNode -- Teach/recording node.
 *
 * Responsibilities:
 *   - Record robot poses during the teach phase.
 *   - Apply SE(3) Frechet mean smoothing when recording stops.
 *   - Publish the trajectory on /wiln/trajectory (transient_local).
 *
 * Commands consumed from /wiln/command:
 *   "start_recording"   -- Begin pose recording (IDLE -> RECORDING).
 *   "stop_recording"    -- Stop, smooth, and publish trajectory (RECORDING -> IDLE).
 *   "smooth_trajectory" -- Re-smooth in place (IDLE only).
 *   "clear_trajectory"  -- Clear all recorded poses (IDLE only).
 */

#include <atomic>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <std_msgs/msg/string.hpp>

#include "wiln/TeachRecorder.hpp"
#include "wiln/msg/wiln_state.hpp"

namespace wiln {

class WilnTeachNode : public rclcpp::Node {
public:
    WilnTeachNode();

private:
    // ----- Core component -----
    std::unique_ptr<TeachRecorder> recorder_;

    // ----- Hot-path state -----
    enum class State : uint8_t { IDLE = 0, RECORDING = 1 };
    std::atomic<State> state_{State::IDLE};
    std::atomic<bool>  driving_forward_{true};

    // ----- ROS interfaces -----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         command_sub_;

    rclcpp::Publisher<norlab_controllers_msgs::msg::PathSequence>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                        global_plan_pub_;
    rclcpp::Publisher<wiln::msg::WilnState>::SharedPtr                       state_pub_;

    // ----- Callbacks -----
    void onOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void onCmdVel(geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void onCommand(std_msgs::msg::String::SharedPtr msg);

    // ----- Command handlers -----
    void handleStartRecording();
    void handleStopRecording();
    void handleSmoothTrajectory();
    void handleClearTrajectory();

    // ----- Helpers -----
    void publishTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj);
    void publishState(uint8_t state_code, const std::string& detail = "");
};

} // namespace wiln
