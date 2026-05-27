#pragma once
/**
 * WilnReplayNode -- Replay hot-path node (10 Hz).
 *
 * Responsibilities:
 *   - On "play" command: set trajectory on the streamer, start 10 Hz loop.
 *   - 10 Hz loop: getLocalHorizon + deform (elastic band) + publish /wiln/control/local_plan.
 *   - Consume pre-processed obstacles from /wiln/obstacles (published by wiln_obstacle_node).
 *   - Publish diagnostics, debug topics, and Foxglove markers.
 *   - On "cancel" or empty horizon: stop cleanly, re-enable SLAM mapping.
 *
 * Commands consumed from /wiln/command:
 *   "play"    -- Start replay (IDLE -> PLAYING).
 *   "cancel"  -- Stop replay (PLAYING -> IDLE).
 */

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wiln/PathDeformer.hpp"
#include "wiln/RobotModel.hpp"
#include "wiln/TrajectoryStreamer.hpp"
#include "wiln/msg/wiln_state.hpp"
#include "wiln/msg/replay_diagnostics.hpp"

namespace wiln {

class WilnReplayNode : public rclcpp::Node {
public:
    WilnReplayNode();

private:
    // ----- Core components -----
    std::unique_ptr<TrajectoryStreamer> streamer_;
    std::unique_ptr<PathDeformer>       deformer_;
    std::shared_ptr<RobotModel>         robot_model_;

    // ----- Hot-path state (PLAYING or IDLE) -----
    enum class State : uint8_t { IDLE = 0, PLAYING = 2 };
    std::atomic<State> state_{State::IDLE};

    // ----- Latest robot pose (lock-free pointer swap) -----
    std::shared_ptr<const geometry_msgs::msg::Pose> current_pose_ptr_;
    std::mutex pose_mutex_;

    // ----- Latest obstacles from /wiln/obstacles -----
    std::vector<Eigen::Vector3d> latest_obstacles_;
    std::mutex obstacles_mutex_;

    // ----- Cached trajectory (latched from /wiln/trajectory) -----
    norlab_controllers_msgs::msg::PathSequence cached_trajectory_;
    std::mutex traj_mutex_;
    bool traj_received_{false};

    // ----- Fallback plan cache (reused when deformer fails) -----
    nav_msgs::msg::Path last_valid_plan_;
    std::mutex          last_valid_plan_mutex_;

    // ----- Callback groups -----
    rclcpp::CallbackGroup::SharedPtr stream_group_;

    // ----- Subscriptions -----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    obstacles_sub_;
    rclcpp::Subscription<norlab_controllers_msgs::msg::PathSequence>::SharedPtr traj_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           articulation_sub_;

    // ----- Publishers -----
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                control_plan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                horizon_debug_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                deformed_debug_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<wiln::msg::WilnState>::SharedPtr               replay_state_pub_;
    rclcpp::Publisher<wiln::msg::ReplayDiagnostics>::SharedPtr       diagnostics_pub_;

    // ----- Mapper clients (async, non-blocking) -----
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr enable_mapping_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr disable_mapping_client_;

    // ----- Timer -----
    rclcpp::TimerBase::SharedPtr stream_timer_;

    // ----- Params -----
    std::string control_local_plan_topic_;
    double      trajectory_speed_{0.40};

    // ----- Callbacks -----
    void onOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void onObstacles(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onTrajectory(norlab_controllers_msgs::msg::PathSequence::SharedPtr msg);
    void onCommand(std_msgs::msg::String::SharedPtr msg);
    void onArticulationAngle(std_msgs::msg::Float64::SharedPtr msg);

    // ----- Hot path -----
    void streamLoop();

    // ----- Command handlers -----
    void handlePlay();
    void handleCancel();
    void stopReplay();

    // ----- Helpers -----
    norlab_controllers_msgs::msg::PathSequence reverseTrajectory(
        const norlab_controllers_msgs::msg::PathSequence& traj) const;

    double distanceToPose(const geometry_msgs::msg::Pose& from,
                          const geometry_msgs::msg::PoseStamped& to) const;

    void publishState(uint8_t state_code, const std::string& detail = "");

    void publishMarkers(const nav_msgs::msg::Path&          horizon,
                        const nav_msgs::msg::Path&          deformed,
                        const std::vector<Eigen::Vector3d>& obstacles,
                        const TrajectoryStreamer::Diag&     stream_diag,
                        const PathDeformer::Diag&           deform_diag);

    static std::vector<Eigen::Vector3d> fromPointCloud2(
        const sensor_msgs::msg::PointCloud2& msg);
};

} // namespace wiln
