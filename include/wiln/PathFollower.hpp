#pragma once
/**
 * PathFollower -- 20 Hz path-following control node.
 *
 * Port of mtt_bringup/mtt_path_follower.py to C++.
 *
 * Responsibilities:
 *   - On "play" command: activate the 20 Hz control loop.
 *   - Follow the trajectory from /wiln/trajectory (PathSequence, latched).
 *   - Prioritize the live local plan from /wiln/control/local_plan for steering.
 *   - Safety-stop if the local plan goes stale after having been received once.
 *   - Abort if tracking errors exceed limits for longer than the grace period.
 *   - On "cancel" or end-of-trajectory: publish zero cmd_vel and stop.
 *
 * Commands consumed from /wiln/command: "play", "cancel"
 * Follows /wiln/replay/state to detect external cancellation by replay_node.
 */

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "wiln/MotionModel.hpp"
#include "wiln/PathProgress.hpp"
#include "wiln/msg/wiln_state.hpp"

namespace wiln {

class PathFollower : public rclcpp::Node {
public:
    PathFollower();

private:
    // ----- Motion model -----
    MotionModelParams motion_params_;

    // ----- Control gains -----
    double k_y_           {0.20};
    double k_theta_       {0.90};
    double kappa_max_     {0.35};
    double lookahead_distance_m_ {1.0};
    int    waypoint_search_ahead_points_ {50};
    double waypoint_tol_  {0.35};
    double final_hdg_tol_ {0.35};

    // ----- Speed -----
    double default_speed_ {0.60};
    double max_speed_     {0.80};
    double min_speed_     {0.25};
    double slowdown_alpha_{2.0};

    // ----- Safety limits -----
    double odom_timeout_s_         {2.5};
    double local_plan_timeout_s_   {0.5};
    double max_lateral_error_m_    {1.25};
    double max_heading_error_rad_  {1.20};
    double max_target_distance_m_  {4.0};
    double tracking_error_grace_s_ {1.0};
    double obstacle_gate_timeout_s_ {0.5};
    double join_max_lateral_error_m_ {3.0};
    double join_max_heading_error_rad_ {1.20};
    double join_capture_lateral_m_ {0.35};
    double join_capture_heading_rad_ {0.30};
    double join_speed_ms_ {0.45};
    double join_timeout_s_ {35.0};

    // ----- Feedforward / adaptive bias -----
    bool   use_path_ff_            {true};
    bool   use_adaptive_bias_      {false};
    double adaptive_kappa_i_gain_  {0.03};
    double adaptive_kappa_decay_   {0.02};
    double adaptive_kappa_deadband_{0.05};
    double adaptive_kappa_limit_   {0.18};

    // ----- Servo modes (MTT-specific) -----
    double psi_dot_max_rad_s_  {0.5};
    double articulation_recenter_s_ {4.0};
    double articulation_center_tolerance_rad_ {0.05};
    double articulation_feedback_timeout_s_ {0.5};
    bool   use_articulation_servo_ {false};
    bool   use_speed_servo_        {false};
    // When true, publish one canonical TwistStamped (signed speed + normalized
    // articulation) and never write either servo setpoint topic directly.
    bool   external_command_mux_   {false};

    // ----- Fallback odom (dead-reckoning bridge when ICP stale) -----
    double fallback_max_s_     {3.0};   // max seconds on fallback before safe stop
    double fallback_history_s_ {20.0};  // encoder history for delayed ICP measurements
    double fallback_anchor_max_skew_s_ {0.15};
    bool   debug_              {false}; // enable verbose [DBG] logs

    // ----- Deadman gate (require deadman held to output non-zero cmd_vel) -----
    bool   require_deadman_    {false};
    bool   deadman_held_       {false};
    std::mutex deadman_mutex_;

    // Ground-truth articulation feedback used to finish recentering early only
    // after the physical joint, not merely the command, reaches center.
    double articulation_feedback_rad_{0.0};
    rclcpp::Time articulation_feedback_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    bool articulation_feedback_received_{false};
    std::mutex articulation_feedback_mutex_;

    // ----- Active-follow state (protected by follow_mutex_) -----
    std::mutex follow_mutex_;
    bool   following_              {false};
    int    current_segment_        {0};
    int    progress_index_         {0};
    int    waypoint_index_         {0};
    double prev_psi_cmd_           {0.0};
    double kappa_adaptive_bias_    {0.0};
    bool   local_plan_rcvd_once_   {false};
    rclcpp::Time path_lost_since_;
    bool   path_lost_tracking_     {false};
    bool   replay_node_playing_    {false};
    bool   recenter_active_        {false};
    std::chrono::steady_clock::time_point recenter_until_{};
    bool   joining_path_           {false};
    rclcpp::Time join_started_at_;

    // Active trajectory segments (set when "play" is received)
    std::vector<norlab_controllers_msgs::msg::DirectionalPath> active_segments_;
    double active_speed_           {0.0};

    // ----- Cached inputs — accepted ICP corrections (/mapping/icp_measurement) -----
    nav_msgs::msg::Odometry latest_odom_;
    rclcpp::Time            odom_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    bool                    odom_received_{false};
    // Encoder odom sampled when latest_odom_ arrived.  The control pose is the
    // accepted ICP pose propagated by the encoder delta from this anchor.
    nav_msgs::msg::Odometry icp_anchor_fallback_odom_;
    bool                    icp_anchor_fallback_valid_{false};
    std::mutex              odom_mutex_;

    // ----- Fallback odom (/mtt_odometry) — dead-reckoning bridge -----
    nav_msgs::msg::Odometry fallback_odom_;
    rclcpp::Time            fallback_odom_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    bool                    fallback_odom_received_{false};
    std::deque<nav_msgs::msg::Odometry> fallback_odom_history_;
    std::mutex              fallback_odom_mutex_;

    // True while the most recent accepted ICP correction is older than the
    // normal freshness threshold and encoder propagation is bridging the gap.
    bool fallback_active_{false};

    nav_msgs::msg::Path latest_local_plan_;
    rclcpp::Time        local_plan_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    std::mutex          local_plan_mutex_;

    norlab_controllers_msgs::msg::PathSequence cached_trajectory_;
    std::mutex traj_mutex_;
    bool traj_received_{false};

    bool obstacle_stop_requested_{false};
    double obstacle_slowdown_scale_{1.0};
    rclcpp::Time obstacle_stop_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    rclcpp::Time obstacle_slowdown_stamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    bool obstacle_stop_received_{false};
    bool obstacle_slowdown_received_{false};
    bool obstacle_hold_state_published_{false};
    std::mutex obstacle_mutex_;

    // ----- Callback group -----
    rclcpp::CallbackGroup::SharedPtr control_group_;

    // ----- Subscriptions -----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         fallback_odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             deadman_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             local_plan_sub_;
    rclcpp::Subscription<norlab_controllers_msgs::msg::PathSequence>::SharedPtr traj_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           command_sub_;
    rclcpp::Subscription<wiln::msg::WilnState>::SharedPtr            replay_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             obstacle_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr          obstacle_slowdown_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr          articulation_feedback_sub_;

    // ----- Publishers -----
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            wiln_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  target_pose_pub_;
    rclcpp::Publisher<wiln::msg::WilnState>::SharedPtr             follower_state_pub_;
    // Debug
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_lateral_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_heading_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_target_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_kappa_desired_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_kappa_ff_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_kappa_bias_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_kappa_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_kappa_eff_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_slip_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_psi_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_psi_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_steering_pub_;
    // MTT servo (optional)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr articulation_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_setpoint_pub_;

    // ----- 20 Hz control timer -----
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ----- Subscription callbacks -----
    void onOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void onFallbackOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void onDeadman(std_msgs::msg::Bool::SharedPtr msg);
    void onLocalPlan(nav_msgs::msg::Path::SharedPtr msg);
    void onTrajectory(norlab_controllers_msgs::msg::PathSequence::SharedPtr msg);
    void onCommand(std_msgs::msg::String::SharedPtr msg);
    void onReplayState(wiln::msg::WilnState::SharedPtr msg);
    void onObstacleStop(std_msgs::msg::Bool::SharedPtr msg);
    void onObstacleSlowdown(std_msgs::msg::Float32::SharedPtr msg);
    void onArticulationFeedback(std_msgs::msg::Float64::SharedPtr msg);

    // ----- Control loop -----
    void controlLoop();

    // ----- Command handlers -----
    void handlePlay();
    void handleCancel();
    void stopFollowing();

    // ----- Control math -----
    struct ControlOutput {
        double linear_x;
        double steering_normalized;
        double psi_cmd;
        double kappa_desired;
        double kappa_command;
        double kappa_effective;
        double slip;
        double psi_raw;
    };

    ControlOutput computeControl(
        const geometry_msgs::msg::Pose& robot_pose,
        double robot_yaw,
        const geometry_msgs::msg::PoseStamped& target,
        bool forward,
        double speed_ref,
        double prev_psi,
        double kappa_ff,
        double kappa_bias,
        double dt) const;

    // ----- Path utilities -----
    static int findStartIndex(
        const std::vector<geometry_msgs::msg::PoseStamped>& poses,
        const geometry_msgs::msg::Pose& robot_pose);

    static norlab_controllers_msgs::msg::PathSequence reverseTrajectory(
        const norlab_controllers_msgs::msg::PathSequence& trajectory);

    double pathCurvature(
        const std::vector<geometry_msgs::msg::PoseStamped>& poses,
        int idx) const;

    bool segmentComplete(
        const geometry_msgs::msg::Pose& robot_pose,
        double robot_yaw,
        const geometry_msgs::msg::PoseStamped& target,
        bool forward,
        bool final_segment) const;

    struct TrackingErrors { double lateral_m; double heading_rad; double distance_m; };
    static TrackingErrors computePathTrackingErrors(
        const geometry_msgs::msg::Pose& robot_pose,
        double robot_yaw,
        const geometry_msgs::msg::PoseStamped& nearest_path_pose,
        bool forward);

    bool trackingErrorExceeded(const TrackingErrors& e) const;

    double updateAdaptiveBias(double lateral_error, double dt, double prev_bias) const;

    // ----- Fallback / dead-reckoning -----
    /// Propagate an accepted ICP pose by the relative encoder-odom motion.
    static geometry_msgs::msg::Pose propagateIcpPose(
        const geometry_msgs::msg::Pose& icp_anchor,
        const geometry_msgs::msg::Pose& fallback_anchor,
        const geometry_msgs::msg::Pose& fallback_now);

    // ----- Publish helpers -----
    void publishZero();
    void startArticulationRecenter();
    void requestReplayCancel();
    void publishCommand(double linear_x, double steering_normalized, double psi_cmd);
    void publishDebug(double lat, double hdg, double dist,
                      double kd, double kff, double kb,
                      double kc, double ke, double slip,
                      double psi_raw, double psi_cmd, double steer);
    void publishFollowerState(uint8_t state_code, const std::string& detail = "");

    static double yawFromPose(const geometry_msgs::msg::Pose& pose);
    static double distXY(const geometry_msgs::msg::Pose& a,
                         const geometry_msgs::msg::Pose& b);
};

} // namespace wiln
