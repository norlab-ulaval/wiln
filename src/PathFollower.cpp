#include "wiln/PathFollower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wiln {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
PathFollower::PathFollower() : Node("wiln_path_follower")
{
    // ----- Parameters -----
    // Topics
    declare_parameter("cmd_vel_topic",      std::string("controller/cmd_vel"));
    declare_parameter("odom_topic",         std::string("/mapping/icp_measurement"));
    declare_parameter("local_plan_topic",   std::string("/wiln/control/local_plan"));
    declare_parameter("trajectory_topic",   std::string("/wiln/trajectory"));
    declare_parameter("command_topic",      std::string("/wiln/command"));
    declare_parameter("replay_state_topic", std::string("/wiln/replay/state"));
    declare_parameter("state_topic",        std::string("/wiln/follower/state"));
    declare_parameter("obstacle_stop_topic", std::string("/mtt_obstacle/stop_requested"));
    declare_parameter("obstacle_slowdown_topic", std::string("/mtt_obstacle/slowdown_scale"));
    declare_parameter("control_rate_hz",    20.0);

    // Gains
    k_y_           = declare_parameter("k_y",         0.20);
    k_theta_       = declare_parameter("k_theta",     0.90);
    kappa_max_     = declare_parameter("kappa_max",   0.35);
    // Retained as a declared compatibility parameter for older YAML files.
    // Progress no longer depends on entering a waypoint-radius gate.
    (void)declare_parameter("advance_distance_m", 0.35);
    lookahead_distance_m_ = declare_parameter("lookahead_distance_m", 1.0);
    waypoint_search_ahead_points_ = declare_parameter("waypoint_search_ahead_points", 50);
    waypoint_tol_  = declare_parameter("waypoint_tolerance_m",        0.35);
    final_hdg_tol_ = declare_parameter("final_heading_tolerance_rad", 0.35);

    // Speed
    default_speed_ = declare_parameter("default_speed_ms",  0.60);
    max_speed_     = declare_parameter("max_speed_ms",       0.80);
    min_speed_     = declare_parameter("min_speed_ms",       0.25);
    slowdown_alpha_= declare_parameter("slowdown_alpha",     2.0);

    // Safety
    odom_timeout_s_         = declare_parameter("odom_timeout_s",           2.5);
    local_plan_timeout_s_   = declare_parameter("local_plan_timeout_s",     0.5);
    max_lateral_error_m_    = declare_parameter("max_lateral_error_m",      1.25);
    max_heading_error_rad_  = declare_parameter("max_heading_error_rad",    1.20);
    max_target_distance_m_  = declare_parameter("max_target_distance_m",    4.0);
    tracking_error_grace_s_ = declare_parameter("tracking_error_grace_s",   1.0);
    obstacle_gate_timeout_s_= declare_parameter("obstacle_gate_timeout_s",   0.5);
    join_max_lateral_error_m_ = declare_parameter("join_max_lateral_error_m", 3.0);
    join_max_heading_error_rad_ = declare_parameter("join_max_heading_error_rad", 1.20);
    join_capture_lateral_m_ = declare_parameter("join_capture_lateral_m", 0.35);
    join_capture_heading_rad_ = declare_parameter("join_capture_heading_rad", 0.30);
    join_speed_ms_ = declare_parameter("join_speed_ms", 0.45);
    join_timeout_s_ = declare_parameter("join_timeout_s", 35.0);

    // Feedforward / adaptive
    use_path_ff_            = declare_parameter("use_path_curvature_feedforward",   true);
    use_adaptive_bias_      = declare_parameter("use_adaptive_curvature_bias",      false);
    adaptive_kappa_i_gain_  = declare_parameter("adaptive_kappa_i_gain",            0.03);
    adaptive_kappa_decay_   = declare_parameter("adaptive_kappa_decay",             0.02);
    adaptive_kappa_deadband_= declare_parameter("adaptive_kappa_deadband_m",        0.05);
    adaptive_kappa_limit_   = declare_parameter("adaptive_kappa_bias_limit",        0.18);

    // Motion model
    motion_params_.wheelbase_m            = declare_parameter("l_eq_m",                      2.4);
    motion_params_.max_articulation_rad   = declare_parameter("psi_max_rad",          M_PI/3.0);
    motion_params_.min_turn_speed_ms      = min_speed_;
    motion_params_.use_slip_heuristic     = declare_parameter("model_use_slip_heuristic",    true);
    motion_params_.yaw_slip_base          = declare_parameter("model_yaw_slip_base",          0.10);
    motion_params_.yaw_slip_speed_gain    = declare_parameter("model_yaw_slip_speed_gain",    0.05);
    motion_params_.yaw_slip_articulation_gain =
        declare_parameter("model_yaw_slip_articulation_gain", 0.15);
    motion_params_.yaw_slip_min_scale     = declare_parameter("model_yaw_slip_min_scale",     0.55);

    psi_dot_max_rad_s_     = declare_parameter("psi_dot_max_rad_s",    0.5);
    articulation_recenter_s_ = declare_parameter("articulation_recenter_s", 8.0);
    articulation_center_tolerance_rad_ =
        declare_parameter("articulation_center_tolerance_rad", 0.05);
    articulation_feedback_timeout_s_ =
        declare_parameter("articulation_feedback_timeout_s", 0.5);
    use_articulation_servo_= declare_parameter("use_articulation_servo", false);
    use_speed_servo_       = declare_parameter("use_speed_servo",        false);
    external_command_mux_  = declare_parameter("external_command_mux",   false);
    fallback_max_s_        = declare_parameter("fallback_max_s",         3.0);
    fallback_history_s_    = declare_parameter("fallback_history_s",     20.0);
    fallback_anchor_max_skew_s_ =
        declare_parameter("fallback_anchor_max_skew_s", 0.15);
    debug_                 = declare_parameter("debug",                  false);
    require_deadman_       = declare_parameter("require_deadman",        false);

    const std::string cmd_vel_topic    = get_parameter("cmd_vel_topic").as_string();
    const std::string odom_topic       = get_parameter("odom_topic").as_string();
    const std::string local_plan_topic = get_parameter("local_plan_topic").as_string();
    const std::string trajectory_topic = get_parameter("trajectory_topic").as_string();
    const std::string command_topic = get_parameter("command_topic").as_string();
    const std::string replay_state_topic = get_parameter("replay_state_topic").as_string();
    const std::string state_topic = get_parameter("state_topic").as_string();
    const std::string obstacle_stop_topic = get_parameter("obstacle_stop_topic").as_string();
    const std::string obstacle_slowdown_topic = get_parameter("obstacle_slowdown_topic").as_string();
    const std::string articulation_feedback_topic = declare_parameter(
        "articulation_feedback_topic", std::string("/hardware/articulation_angle"));
    const std::string articulation_setpoint_topic = declare_parameter(
        "articulation_setpoint_topic", std::string("/mtt_articulation_setpoint"));
    const std::string speed_setpoint_topic = declare_parameter(
        "speed_setpoint_topic", std::string("/speed_setpoint"));
    const double control_rate_hz       = get_parameter("control_rate_hz").as_double();

    // ----- Callback groups -----
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions ctrl_opts;
    ctrl_opts.callback_group = control_group_;

    // ----- QoS -----
    auto be_qos        = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
    auto rel_qos       = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    // ----- Subscriptions -----
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, be_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr m) { onOdom(m); },
        ctrl_opts);

    {
        const std::string fallback_odom_topic = declare_parameter("fallback_odom_topic", std::string(""));
        if (!fallback_odom_topic.empty()) {
            fallback_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                fallback_odom_topic, be_qos,
                [this](nav_msgs::msg::Odometry::SharedPtr m) { onFallbackOdom(m); },
                ctrl_opts);
            RCLCPP_INFO(get_logger(), "Fallback odom enabled: %s (max %.1f s)",
                fallback_odom_topic.c_str(), fallback_max_s_);
        }
    }

    if (require_deadman_) {
        const std::string deadman_topic = declare_parameter("deadman_topic",
            std::string("mtt_control/teleop_deadman"));
        deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
            deadman_topic, rel_qos,
            [this](std_msgs::msg::Bool::SharedPtr m) { onDeadman(m); });
        RCLCPP_INFO(get_logger(), "Deadman gate enabled: topic=%s", deadman_topic.c_str());
    } else {
        declare_parameter("deadman_topic", std::string("mtt_control/teleop_deadman"));
    }

    local_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
        local_plan_topic, rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        [this](nav_msgs::msg::Path::SharedPtr m) { onLocalPlan(m); });

    traj_sub_ = create_subscription<norlab_controllers_msgs::msg::PathSequence>(
        trajectory_topic, transient_qos,
        [this](norlab_controllers_msgs::msg::PathSequence::SharedPtr m) { onTrajectory(m); });

    command_sub_ = create_subscription<std_msgs::msg::String>(
        command_topic, rel_qos,
        [this](std_msgs::msg::String::SharedPtr m) { onCommand(m); });

    replay_state_sub_ = create_subscription<wiln::msg::WilnState>(
        replay_state_topic, transient_qos,
        [this](wiln::msg::WilnState::SharedPtr m) { onReplayState(m); });

    obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        obstacle_stop_topic, rel_qos,
        [this](std_msgs::msg::Bool::SharedPtr m) { onObstacleStop(m); });

    obstacle_slowdown_sub_ = create_subscription<std_msgs::msg::Float32>(
        obstacle_slowdown_topic, rel_qos,
        [this](std_msgs::msg::Float32::SharedPtr m) { onObstacleSlowdown(m); });

    articulation_feedback_sub_ = create_subscription<std_msgs::msg::Float64>(
        articulation_feedback_topic, be_qos,
        [this](std_msgs::msg::Float64::SharedPtr m) { onArticulationFeedback(m); });

    // ----- Publishers -----
    cmd_pub_         = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, rel_qos);
    wiln_command_pub_= create_publisher<std_msgs::msg::String>(command_topic, rel_qos);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/target_pose", be_qos);
    follower_state_pub_ = create_publisher<wiln::msg::WilnState>(state_topic, transient_qos);

    debug_lateral_pub_    = create_publisher<std_msgs::msg::Float64>("~/debug/lateral_error_m",       20);
    debug_heading_pub_    = create_publisher<std_msgs::msg::Float64>("~/debug/heading_error_rad",      20);
    debug_target_dist_pub_= create_publisher<std_msgs::msg::Float64>("~/debug/target_distance_m",     20);
    debug_kappa_desired_pub_= create_publisher<std_msgs::msg::Float64>("~/debug/kappa_desired_m_inv", 20);
    debug_kappa_ff_pub_   = create_publisher<std_msgs::msg::Float64>("~/debug/kappa_feedforward_m_inv", 20);
    debug_kappa_bias_pub_ = create_publisher<std_msgs::msg::Float64>("~/debug/kappa_adaptive_bias_m_inv", 20);
    debug_kappa_cmd_pub_  = create_publisher<std_msgs::msg::Float64>("~/debug/kappa_command_m_inv",   20);
    debug_kappa_eff_pub_  = create_publisher<std_msgs::msg::Float64>("~/debug/kappa_effective_est_m_inv", 20);
    debug_slip_pub_       = create_publisher<std_msgs::msg::Float64>("~/debug/slip_scale",            20);
    debug_psi_raw_pub_    = create_publisher<std_msgs::msg::Float64>("~/debug/psi_raw_rad",           20);
    debug_psi_cmd_pub_    = create_publisher<std_msgs::msg::Float64>("~/debug/psi_cmd_rad",           20);
    debug_steering_pub_   = create_publisher<std_msgs::msg::Float64>("~/debug/steering_normalized",   20);

    if (use_articulation_servo_ && !external_command_mux_)
        articulation_pub_ = create_publisher<std_msgs::msg::Float64>(articulation_setpoint_topic, 20);
    if (use_speed_servo_ && !external_command_mux_)
        speed_setpoint_pub_ = create_publisher<std_msgs::msg::Float64>(speed_setpoint_topic, 20);

    // ----- 20 Hz control timer -----
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz));
    control_timer_ = create_wall_timer(period_ns, [this]() { controlLoop(); }, control_group_);

    publishFollowerState(wiln::msg::WilnState::IDLE, "ready");
    RCLCPP_INFO(get_logger(),
        "wiln_path_follower started. odom=%s trajectory=%s command=%s cmd_vel=%s rate=%.0f Hz, speed=%.2f m/s, "
        "local_plan_topic=%s, obstacle_stop=%s, obstacle_slowdown=%s, psi_max=%.1f deg, "
        "slip=%s, external_command_mux=%s",
        odom_topic.c_str(), trajectory_topic.c_str(), command_topic.c_str(), cmd_vel_topic.c_str(),
        control_rate_hz, default_speed_, local_plan_topic.c_str(),
        obstacle_stop_topic.c_str(), obstacle_slowdown_topic.c_str(),
        motion_params_.max_articulation_rad * 180.0 / M_PI,
        motion_params_.use_slip_heuristic ? "on" : "off",
        external_command_mux_ ? "on" : "off");
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------
void PathFollower::onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    nav_msgs::msg::Odometry fallback_at_icp;
    bool fallback_available = false;
    double anchor_skew_s = std::numeric_limits<double>::infinity();
    {
        std::lock_guard<std::mutex> fk(fallback_odom_mutex_);
        const rclcpp::Time icp_time(msg->header.stamp);
        if (icp_time.nanoseconds() == 0 && fallback_odom_received_) {
            // Header-less input is not expected, but using the current encoder
            // pose is preferable to silently accepting an unanchored pose.
            fallback_at_icp = fallback_odom_;
            fallback_available = true;
            anchor_skew_s = 0.0;
        } else {
            for (const auto& candidate : fallback_odom_history_) {
                const double skew = std::abs(
                    (rclcpp::Time(candidate.header.stamp) - icp_time).seconds());
                if (skew < anchor_skew_s) {
                    anchor_skew_s = skew;
                    fallback_at_icp = candidate;
                }
            }
            fallback_available = anchor_skew_s <= fallback_anchor_max_skew_s_;
        }
    }

    if (!fallback_available) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Ignoring accepted ICP pose: no encoder sample within %.0f ms of scan stamp (best %.0f ms).",
            fallback_anchor_max_skew_s_ * 1000.0, anchor_skew_s * 1000.0);
        return;
    }

    std::lock_guard<std::mutex> lk(odom_mutex_);
    latest_odom_  = *msg;
    odom_stamp_   = now();
    odom_received_= true;
    if (fallback_available) {
        icp_anchor_fallback_odom_ = fallback_at_icp;
        icp_anchor_fallback_valid_ = true;
    }
}

void PathFollower::onFallbackOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(fallback_odom_mutex_);
    fallback_odom_          = *msg;
    fallback_odom_stamp_    = now();
    fallback_odom_received_ = true;

    const rclcpp::Time sample_time(msg->header.stamp);
    if (!fallback_odom_history_.empty()) {
        const rclcpp::Time newest_time(fallback_odom_history_.back().header.stamp);
        if (sample_time < newest_time) {
            // Clock reset or an odom publisher restart: old samples cannot be
            // matched safely to future ICP measurements.
            fallback_odom_history_.clear();
        }
    }
    fallback_odom_history_.push_back(*msg);
    while (fallback_odom_history_.size() > 1) {
        const rclcpp::Time oldest_time(fallback_odom_history_.front().header.stamp);
        if ((sample_time - oldest_time).seconds() <= fallback_history_s_) break;
        fallback_odom_history_.pop_front();
    }
}

void PathFollower::onDeadman(std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(deadman_mutex_);
    deadman_held_ = msg->data;
}

void PathFollower::onLocalPlan(nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(local_plan_mutex_);
    latest_local_plan_ = *msg;
    local_plan_stamp_  = now();
}

void PathFollower::onTrajectory(
    norlab_controllers_msgs::msg::PathSequence::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(traj_mutex_);
    cached_trajectory_ = *msg;
    traj_received_     = true;
}

void PathFollower::onCommand(std_msgs::msg::String::SharedPtr msg)
{
    const std::string& cmd = msg->data;
    if (cmd == "play")        handlePlay();
    else if (cmd == "cancel") handleCancel();
}

void PathFollower::onReplayState(wiln::msg::WilnState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(follow_mutex_);
    replay_node_playing_ = msg->state == wiln::msg::WilnState::PLAYING;
    // If replay_node goes IDLE externally (trajectory completed, mapper-freeze
    // refusal, or cancellation), stop the independent follower as well.
    if (msg->state == wiln::msg::WilnState::IDLE) {
        if (following_) {
            following_ = false;
            startArticulationRecenter();
            publishZero();
            publishFollowerState(wiln::msg::WilnState::IDLE, "replay completed");
        }
    }
}

void PathFollower::onObstacleStop(std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(obstacle_mutex_);
    obstacle_stop_requested_ = msg->data;
    obstacle_stop_stamp_ = now();
    obstacle_stop_received_ = true;
}

void PathFollower::onObstacleSlowdown(std_msgs::msg::Float32::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(obstacle_mutex_);
    obstacle_slowdown_scale_ = mmClamp(static_cast<double>(msg->data), 0.0, 1.0);
    obstacle_slowdown_stamp_ = now();
    obstacle_slowdown_received_ = true;
}

void PathFollower::onArticulationFeedback(std_msgs::msg::Float64::SharedPtr msg)
{
    if (!std::isfinite(msg->data)) {
        return;
    }
    std::lock_guard<std::mutex> lk(articulation_feedback_mutex_);
    articulation_feedback_rad_ = msg->data;
    articulation_feedback_stamp_ = now();
    articulation_feedback_received_ = true;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------
void PathFollower::handlePlay()
{
    {
        std::lock_guard<std::mutex> lk(follow_mutex_);
        if (following_ || replay_node_playing_) {
            RCLCPP_WARN(get_logger(),
                "Ignoring duplicate play command while replay is already armed or active.");
            return;
        }
    }

    norlab_controllers_msgs::msg::PathSequence traj;
    {
        std::lock_guard<std::mutex> lk(traj_mutex_);
        if (!traj_received_ || cached_trajectory_.paths.empty()) {
            RCLCPP_WARN(get_logger(), "play: no trajectory available.");
            publishFollowerState(
                wiln::msg::WilnState::IDLE, "play refused: no trajectory");
            return;
        }
        traj = cached_trajectory_;
    }

    // Collect non-empty segments
    std::vector<norlab_controllers_msgs::msg::DirectionalPath> segments;
    for (const auto& seg : traj.paths)
        if (!seg.poses.empty()) segments.push_back(seg);

    if (segments.empty()) {
        RCLCPP_WARN(get_logger(), "play: trajectory has no non-empty segments.");
        publishFollowerState(
            wiln::msg::WilnState::IDLE, "play refused: trajectory has no poses");
        return;
    }

    // Start from an accepted ICP correction propagated to the current encoder
    // odom.  Raw fallback odom is in a different drifting frame and must never
    // be used directly as a map-frame pose.
    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::Odometry anchor_fallback;
    bool primary_received = false;
    bool anchor_valid = false;
    rclcpp::Time primary_stamp{0, 0, get_clock()->get_clock_type()};
    {
        std::lock_guard<std::mutex> ok(odom_mutex_);
        odom = latest_odom_;
        anchor_fallback = icp_anchor_fallback_odom_;
        primary_received = odom_received_;
        anchor_valid = icp_anchor_fallback_valid_;
        primary_stamp = odom_stamp_;
    }
    nav_msgs::msg::Odometry fallback_now;
    bool fb_fresh = false;
    {
        std::lock_guard<std::mutex> fk(fallback_odom_mutex_);
        fb_fresh = fallback_odom_received_ &&
                   (now() - fallback_odom_stamp_).seconds() <= odom_timeout_s_;
        fallback_now = fallback_odom_;
    }
    const double primary_age = primary_received
        ? (now() - primary_stamp).seconds()
        : std::numeric_limits<double>::infinity();
    if (!primary_received || primary_age > fallback_max_s_ || !anchor_valid || !fb_fresh) {
        RCLCPP_WARN(get_logger(),
            "play: no usable accepted ICP anchor (age=%.2fs, anchor=%s, fallback=%s); refusing to follow.",
            primary_age, anchor_valid ? "yes" : "no", fb_fresh ? "fresh" : "stale");
        publishZero();
        publishFollowerState(wiln::msg::WilnState::IDLE, "play refused: no accepted ICP anchor");
        return;
    }
    odom.pose.pose = propagateIcpPose(
        odom.pose.pose, anchor_fallback.pose.pose, fallback_now.pose.pose);

    // WilnReplayNode chooses the closest endpoint and reverses when replay starts
    // near the taught end. The follower owns a separate cached trajectory, so it
    // must make the same deterministic choice before constructing active segments.
    const double dist_to_start = distXY(odom.pose.pose, segments.front().poses.front().pose);
    const double dist_to_end = distXY(odom.pose.pose, segments.back().poses.back().pose);
    if (dist_to_end < dist_to_start) {
        traj = reverseTrajectory(traj);
        segments.clear();
        for (const auto& seg : traj.paths) {
            if (!seg.poses.empty()) segments.push_back(seg);
        }
        RCLCPP_INFO(get_logger(),
            "Follower closer to end (%.2fm < %.2fm) — using reversed trajectory.",
            dist_to_end, dist_to_start);
    }

    // Determine speed from trajectory (fallback to default)
    double speed = default_speed_;

    std::lock_guard<std::mutex> lk(follow_mutex_);
    active_segments_      = std::move(segments);
    active_speed_         = speed;
    current_segment_      = 0;
    following_            = true;
    recenter_active_      = false;
    local_plan_rcvd_once_ = false;
    path_lost_tracking_   = false;
    fallback_active_      = primary_age > odom_timeout_s_;
    obstacle_hold_state_published_ = false;
    prev_psi_cmd_         = 0.0;
    kappa_adaptive_bias_  = 0.0;
    joining_path_         = true;
    join_started_at_      = now();

    progress_index_ = findStartIndex(active_segments_[0].poses, odom.pose.pose);
    const auto initial_progress = selectPathProgress(
        active_segments_[0].poses,
        progress_index_,
        odom.pose.pose,
        lookahead_distance_m_,
        waypoint_search_ahead_points_);
    progress_index_ = initial_progress.nearest_index;
    waypoint_index_ = initial_progress.target_index;

    if (debug_) {
        RCLCPP_INFO(get_logger(),
            "[DBG] handlePlay: odom=(%.2f,%.2f,yaw=%.1f°) start_wp=%d/%zu segs=%zu speed=%.2f",
            odom.pose.pose.position.x, odom.pose.pose.position.y,
            yawFromPose(odom.pose.pose) * 180.0 / M_PI,
            waypoint_index_,
            active_segments_[0].poses.size() - 1,
            active_segments_.size(), active_speed_);
    }

    publishFollowerState(wiln::msg::WilnState::PLAYING, "following");
    RCLCPP_INFO(get_logger(), "Following trajectory (%zu segments, %.2f m/s).",
        active_segments_.size(), active_speed_);
}

void PathFollower::handleCancel()
{
    std::lock_guard<std::mutex> lk(follow_mutex_);
    startArticulationRecenter();
    following_ = false;
    publishZero();
    publishFollowerState(wiln::msg::WilnState::IDLE, "cancelled");
    RCLCPP_INFO(get_logger(), "Path following cancelled.");
}

void PathFollower::stopFollowing()
{
    // Called from controlLoop — follow_mutex_ is held by caller.
    following_ = false;
    startArticulationRecenter();
    publishZero();
    publishFollowerState(wiln::msg::WilnState::IDLE, "trajectory completed");
    // Keep WilnReplayNode and this follower in the same state. Without this,
    // a follower path-loss left replay_node PLAYING, and a later play command
    // restarted only the follower with a newly selected direction.
    requestReplayCancel();
}

// ---------------------------------------------------------------------------
// 20 Hz control loop
// ---------------------------------------------------------------------------
void PathFollower::controlLoop()
{
    std::lock_guard<std::mutex> lk(follow_mutex_);
    if (!following_) {
        if (recenter_active_) {
            if (std::chrono::steady_clock::now() < recenter_until_) {
                publishZero();
                bool centered = false;
                {
                    std::lock_guard<std::mutex> ak(articulation_feedback_mutex_);
                    centered = articulation_feedback_received_ &&
                        (now() - articulation_feedback_stamp_).seconds() <=
                            articulation_feedback_timeout_s_ &&
                        std::abs(articulation_feedback_rad_) <=
                            articulation_center_tolerance_rad_;
                }
                if (centered) {
                    recenter_active_ = false;
                    RCLCPP_INFO(get_logger(),
                        "Physical articulation reached center after replay stop.");
                }
            } else {
                recenter_active_ = false;
                RCLCPP_WARN(get_logger(),
                    "Articulation recenter timeout after %.1fs; center command stopped.",
                    articulation_recenter_s_);
            }
        }
        return;
    }

    // wiln_replay_node publishes PLAYING only after /mapping/disable_mapping
    // responds. The follower receives the play command independently, so hold
    // zero here until map insertion is confirmed frozen.
    if (!replay_node_playing_) {
        publishZero();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Waiting for mapper freeze acknowledgement before replay motion.");
        return;
    }

    bool obstacle_stop = false;
    double obstacle_slowdown = 1.0;
    {
        std::lock_guard<std::mutex> ok(obstacle_mutex_);
        if (obstacle_stop_received_ && (now() - obstacle_stop_stamp_).seconds() <= obstacle_gate_timeout_s_) {
            obstacle_stop = obstacle_stop_requested_;
        }
        if (obstacle_slowdown_received_ && (now() - obstacle_slowdown_stamp_).seconds() <= obstacle_gate_timeout_s_) {
            obstacle_slowdown = obstacle_slowdown_scale_;
        }
    }
    if (obstacle_stop || obstacle_slowdown <= 0.01) {
        publishZero();
        if (!obstacle_hold_state_published_) {
            publishFollowerState(wiln::msg::WilnState::PLAYING, "paused: front obstacle");
            obstacle_hold_state_published_ = true;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Front obstacle stop active — holding zero cmd_vel and keeping replay alive.");
        return;
    }
    if (obstacle_hold_state_published_) {
        publishFollowerState(wiln::msg::WilnState::PLAYING, "following");
        obstacle_hold_state_published_ = false;
    }

    // --- Deadman gate: if required, hold zero cmd_vel when operator is not present ---
    if (require_deadman_) {
        bool held = false;
        {
            std::lock_guard<std::mutex> dk(deadman_mutex_);
            held = deadman_held_;
        }
        if (!held) {
            publishZero();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Deadman not held — holding zero cmd_vel (trajectory still armed).");
            return;
        }
    }

    // --- Continuous pose estimate: accepted ICP anchor + encoder delta ---
    nav_msgs::msg::Odometry primary_odom;
    nav_msgs::msg::Odometry anchor_fallback_odom;
    bool primary_received = false;
    bool anchor_valid = false;
    rclcpp::Time primary_stamp{0, 0, get_clock()->get_clock_type()};
    {
        std::lock_guard<std::mutex> ok(odom_mutex_);
        primary_odom  = latest_odom_;
        anchor_fallback_odom = icp_anchor_fallback_odom_;
        primary_received = odom_received_;
        anchor_valid = icp_anchor_fallback_valid_;
        primary_stamp = odom_stamp_;
    }
    nav_msgs::msg::Odometry fallback_now;
    bool fb_fresh = false;
    {
        std::lock_guard<std::mutex> fk(fallback_odom_mutex_);
        fb_fresh = fallback_odom_received_ && (now() - fallback_odom_stamp_).seconds() <= odom_timeout_s_;
        fallback_now = fallback_odom_;
    }

    const double primary_age = primary_received
        ? (now() - primary_stamp).seconds()
        : std::numeric_limits<double>::infinity();
    if (!primary_received || !anchor_valid || !fb_fresh || primary_age > fallback_max_s_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
            "Pose estimate unavailable: accepted ICP age=%.2fs (max %.2fs), anchor=%s, fallback=%s — stopping.",
            primary_age, fallback_max_s_, anchor_valid ? "yes" : "no",
            fb_fresh ? "fresh" : "stale");
        stopFollowing();
        return;
    }

    const bool bridging = primary_age > odom_timeout_s_;
    if (bridging != fallback_active_) {
        fallback_active_ = bridging;
        if (bridging) {
            RCLCPP_WARN(get_logger(),
                "Accepted ICP correction stale; continuing from its anchor with encoder odom (max %.1fs).",
                fallback_max_s_);
        } else {
            RCLCPP_INFO(get_logger(), "Accepted ICP correction recovered; encoder anchor refreshed.");
        }
    }
    geometry_msgs::msg::Pose robot_pose = propagateIcpPose(
        primary_odom.pose.pose,
        anchor_fallback_odom.pose.pose,
        fallback_now.pose.pose);

    const double robot_yaw = yawFromPose(robot_pose);

    auto& segment = active_segments_[current_segment_];
    const bool forward       = segment.forward;
    const bool final_segment = (current_segment_ == static_cast<int>(active_segments_.size()) - 1);

    // --- Recover monotone progress and select an arc-length lookahead target ---
    // This deliberately does not require the robot to enter a tiny waypoint
    // radius.  A missed sample is skipped once a later sample is closer, so a
    // waypoint behind the robot can never make the controller turn back.
    const auto global_progress = selectPathProgress(
        segment.poses,
        progress_index_,
        robot_pose,
        lookahead_distance_m_,
        waypoint_search_ahead_points_);
    progress_index_ = global_progress.nearest_index;
    waypoint_index_ = global_progress.target_index;
    const auto& global_target = segment.poses[waypoint_index_];

    if (debug_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[DBG] seg=%d wp=%d/%zu robot=(%.2f,%.2f,yaw=%.1f°) fwd=%d",
            current_segment_, waypoint_index_,
            static_cast<size_t>(segment.poses.size() - 1),
            robot_pose.position.x, robot_pose.position.y, robot_yaw * 180.0 / M_PI,
            static_cast<int>(forward));
    }

    // --- Choose steering target (local plan or global fallback) ---
    nav_msgs::msg::Path local_plan;
    bool local_plan_fresh = false;
    {
        std::lock_guard<std::mutex> lk2(local_plan_mutex_);
        local_plan = latest_local_plan_;
        local_plan_fresh = !local_plan.poses.empty() &&
            (now() - local_plan_stamp_).seconds() < local_plan_timeout_s_;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::PoseStamped nearest_path_pose;
    std::vector<geometry_msgs::msg::PoseStamped>* active_poses = nullptr;
    int active_index = 0;

    if (local_plan_fresh && !local_plan.poses.empty()) {
        local_plan_rcvd_once_ = true;
        const int local_nearest = findStartIndex(local_plan.poses, robot_pose);
        const auto local_progress = selectPathProgress(
            local_plan.poses,
            local_nearest,
            robot_pose,
            lookahead_distance_m_,
            waypoint_search_ahead_points_);
        const int li = local_progress.target_index;
        target_pose  = local_plan.poses[li];
        nearest_path_pose = local_plan.poses[local_progress.nearest_index];
        active_poses = &local_plan.poses;
        active_index = li;
        if (debug_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "[DBG] using LOCAL PLAN idx=%d/%zu", li, local_plan.poses.size() - 1);
        }
    } else if (local_plan_rcvd_once_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Local plan stale — holding zero cmd_vel.");
        publishZero();
        return;
    } else {
        // Fallback to global path before first local plan is received
        target_pose  = global_target;
        nearest_path_pose = segment.poses[global_progress.nearest_index];
        active_poses = &segment.poses;
        active_index = waypoint_index_;
        if (debug_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "[DBG] using GLOBAL PATH wp=%d tgt=(%.2f,%.2f,yaw=%.1f°)",
                waypoint_index_,
                target_pose.pose.position.x, target_pose.pose.position.y,
                yawFromPose(target_pose.pose) * 180.0 / M_PI);
        }
    }

    // --- Publish target pose for debug ---
    target_pose_pub_->publish(target_pose);

    // --- Tracking errors relative to the path, not to the lookahead point ---
    // Longitudinal lookahead is intentional and must not be misclassified as
    // path loss.  Cross-track distance and heading are measured at the nearest
    // monotone path sample; the lookahead target is used only for control.
    auto errs = computePathTrackingErrors(
        robot_pose, robot_yaw, nearest_path_pose, forward);
    if (debug_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[DBG] tracking e_y=%.2fm e_th=%.1f° dist=%.2fm (limits: lat=%.1f head=%.1f°)",
            errs.lateral_m, errs.heading_rad * 180.0 / M_PI, errs.distance_m,
            max_lateral_error_m_, max_heading_error_rad_ * 180.0 / M_PI);
    }

    const double join_elapsed_s = (now() - join_started_at_).seconds();
    if (joining_path_ &&
        std::abs(errs.lateral_m) <= join_capture_lateral_m_ &&
        std::abs(errs.heading_rad) <= join_capture_heading_rad_)
    {
        joining_path_ = false;
        path_lost_tracking_ = false;
        RCLCPP_INFO(get_logger(),
            "Route captured after %.2fs: lateral=%.2fm heading=%.1fdeg progress=%d/%zu.",
            join_elapsed_s,
            errs.lateral_m,
            errs.heading_rad * 180.0 / M_PI,
            progress_index_,
            segment.poses.size() - 1);
    }

    if (joining_path_) {
        const bool outside_join_envelope =
            std::abs(errs.lateral_m) > join_max_lateral_error_m_ ||
            std::abs(errs.heading_rad) > join_max_heading_error_rad_;
        if (outside_join_envelope || join_elapsed_s > join_timeout_s_) {
            RCLCPP_ERROR(get_logger(),
                "Unable to capture route: lateral=%.2fm (max %.2f), heading=%.1fdeg (max %.1f), elapsed=%.1fs (max %.1f).",
                errs.lateral_m,
                join_max_lateral_error_m_,
                errs.heading_rad * 180.0 / M_PI,
                join_max_heading_error_rad_ * 180.0 / M_PI,
                join_elapsed_s,
                join_timeout_s_);
            stopFollowing();
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Capturing route: lateral=%.2fm heading=%.1fdeg nearest=%d target=%d speed<=%.2fm/s.",
            errs.lateral_m,
            errs.heading_rad * 180.0 / M_PI,
            progress_index_,
            waypoint_index_,
            join_speed_ms_);
    } else if (trackingErrorExceeded(errs)) {
        if (!path_lost_tracking_) {
            path_lost_since_    = now();
            path_lost_tracking_ = true;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Tracking error high: e_y=%.2f m, e_th=%.1f deg, d=%.2f m",
            errs.lateral_m, errs.heading_rad * 180.0/M_PI, errs.distance_m);
        if ((now() - path_lost_since_).seconds() > tracking_error_grace_s_) {
            RCLCPP_ERROR(get_logger(), "Path lost for too long — aborting.");
            stopFollowing();
            return;
        }
    } else {
        path_lost_tracking_ = false;
    }

    // --- Segment completion ---
    if (waypoint_index_ == static_cast<int>(segment.poses.size()) - 1 &&
        segmentComplete(robot_pose, robot_yaw, global_target, forward, final_segment))
    {
        if (final_segment) {
            if (debug_) {
                RCLCPP_INFO(get_logger(),
                    "[DBG] Trajectory completed. robot=(%.2f,%.2f) target=(%.2f,%.2f) dist=%.2fm",
                    robot_pose.position.x, robot_pose.position.y,
                    global_target.pose.position.x, global_target.pose.position.y,
                    distXY(robot_pose, global_target.pose));
            }
            RCLCPP_INFO(get_logger(), "Trajectory completed.");
            stopFollowing();
            return;
        } else {
            current_segment_++;
            progress_index_ = findStartIndex(
                active_segments_[current_segment_].poses, robot_pose);
            waypoint_index_ = progress_index_;
            joining_path_ = true;
            join_started_at_ = now();
            RCLCPP_INFO(get_logger(), "Advancing to segment %d.", current_segment_);
            return;
        }
    }

    // --- Feedforward curvature ---
    double kappa_ff = pathCurvature(*active_poses, active_index);

    // --- Adaptive bias update ---
    constexpr double dt = 1.0 / 20.0;
    kappa_adaptive_bias_ = updateAdaptiveBias(errs.lateral_m, dt, kappa_adaptive_bias_);

    // --- Compute control ---
    const double commanded_speed = joining_path_
        ? std::min(active_speed_, join_speed_ms_)
        : active_speed_;
    auto ctrl = computeControl(
        robot_pose, robot_yaw, target_pose, forward,
        commanded_speed, prev_psi_cmd_,
        kappa_ff, kappa_adaptive_bias_, dt);

    prev_psi_cmd_ = ctrl.psi_cmd;
    ctrl.linear_x *= obstacle_slowdown;
    publishCommand(ctrl.linear_x, ctrl.steering_normalized, ctrl.psi_cmd);

    publishDebug(errs.lateral_m, errs.heading_rad, errs.distance_m,
                 ctrl.kappa_desired, kappa_ff, kappa_adaptive_bias_,
                 ctrl.kappa_command, ctrl.kappa_effective, ctrl.slip,
                 ctrl.psi_raw, ctrl.psi_cmd, ctrl.steering_normalized);
}

// ---------------------------------------------------------------------------
// Control math
// ---------------------------------------------------------------------------
PathFollower::ControlOutput PathFollower::computeControl(
    const geometry_msgs::msg::Pose& robot_pose,
    double robot_yaw,
    const geometry_msgs::msg::PoseStamped& target,
    bool forward,
    double speed_ref,
    double prev_psi,
    double kappa_ff,
    double kappa_bias,
    double dt) const
{
    const double theta_eff  = forward ? robot_yaw : wrapToPi(robot_yaw + M_PI);
    const double tgt_yaw    = yawFromPose(target.pose);
    const double tgt_eff    = forward ? tgt_yaw : wrapToPi(tgt_yaw + M_PI);

    const double dx = target.pose.position.x - robot_pose.position.x;
    const double dy = target.pose.position.y - robot_pose.position.y;
    const double e_y     = -std::sin(theta_eff)*dx + std::cos(theta_eff)*dy;
    const double e_theta = wrapToPi(tgt_eff - theta_eff);

    double kappa_fb      = k_y_ * e_y + k_theta_ * e_theta;
    double kappa_desired = mmClamp(kappa_ff + kappa_fb + kappa_bias, -kappa_max_, kappa_max_);
    double kappa_actual  = forward ? kappa_desired : -kappa_desired;

    double slip         = slipScale(std::abs(speed_ref), prev_psi, motion_params_);
    double kappa_cmd    = mmClamp(kappa_actual / std::max(slip, 1e-6), -kappa_max_, kappa_max_);

    double psi_raw      = articulationFromCurvature(kappa_cmd, motion_params_);
    double max_delta    = psi_dot_max_rad_s_ * std::max(dt, 1e-3);
    double psi_cmd      = mmClamp(psi_raw, prev_psi - max_delta, prev_psi + max_delta);
    double steer_norm   = normalizedSteerFromArticulation(psi_cmd, motion_params_);
    double kappa_eff    = std::tan(psi_cmd) / std::max(motion_params_.wheelbase_m, 1e-6) * slip;

    double speed_mag    = std::min(std::abs(speed_ref), max_speed_);
    speed_mag           = speed_mag / (1.0 + slowdown_alpha_ * std::abs(kappa_desired));
    speed_mag           = std::max(speed_mag, min_speed_);
    double linear_x     = forward ? speed_mag : -speed_mag;

    return {linear_x, steer_norm, psi_cmd,
            kappa_desired, kappa_cmd, kappa_eff, slip, psi_raw};
}

// ---------------------------------------------------------------------------
// Path utilities
// ---------------------------------------------------------------------------
int PathFollower::findStartIndex(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    const geometry_msgs::msg::Pose& robot_pose)
{
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(poses.size()); ++i) {
        double d = distXY(robot_pose, poses[i].pose);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

norlab_controllers_msgs::msg::PathSequence PathFollower::reverseTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& trajectory)
{
    auto reversed = trajectory;
    std::reverse(reversed.paths.begin(), reversed.paths.end());

    tf2::Quaternion half_turn;
    half_turn.setRPY(0.0, 0.0, M_PI);
    for (auto& path : reversed.paths) {
        std::reverse(path.poses.begin(), path.poses.end());
        path.forward = !path.forward;
        for (auto& pose_stamped : path.poses) {
            tf2::Quaternion orientation;
            tf2::fromMsg(pose_stamped.pose.orientation, orientation);
            orientation = half_turn * orientation;
            orientation.normalize();
            pose_stamped.pose.orientation = tf2::toMsg(orientation);
        }
    }
    return reversed;
}

double PathFollower::pathCurvature(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses, int idx) const
{
    if (!use_path_ff_ || static_cast<int>(poses.size()) < 3) return 0.0;
    int i0 = std::max(idx-1, 0);
    int i1 = std::max(std::min(idx, static_cast<int>(poses.size())-1), 0);
    int i2 = std::min(idx+1, static_cast<int>(poses.size())-1);
    if (i0==i1) { i2=std::min(i1+2,static_cast<int>(poses.size())-1); }
    if (i1==i2) { i0=std::max(i1-2,0); }
    if (i0==i1 || i1==i2) return 0.0;

    const auto& p0 = poses[i0].pose.position;
    const auto& p1 = poses[i1].pose.position;
    const auto& p2 = poses[i2].pose.position;
    double a = std::hypot(p1.x-p0.x, p1.y-p0.y);
    double b = std::hypot(p2.x-p1.x, p2.y-p1.y);
    double c = std::hypot(p2.x-p0.x, p2.y-p0.y);
    double denom = std::max(a*b*c, 1e-9);
    double area2 = (p1.x-p0.x)*(p2.y-p0.y) - (p1.y-p0.y)*(p2.x-p0.x);
    return mmClamp(2.0*area2/denom, -kappa_max_, kappa_max_);
}

bool PathFollower::segmentComplete(
    const geometry_msgs::msg::Pose& robot_pose, double robot_yaw,
    const geometry_msgs::msg::PoseStamped& target,
    bool forward, bool final_segment) const
{
    if (distXY(robot_pose, target.pose) > waypoint_tol_) return false;
    if (!final_segment) return true;
    double tgt_yaw  = yawFromPose(target.pose);
    double tgt_eff  = forward ? tgt_yaw : wrapToPi(tgt_yaw + M_PI);
    double hdg_err  = wrapToPi(tgt_eff - (forward ? robot_yaw : wrapToPi(robot_yaw + M_PI)));
    return std::abs(hdg_err) <= final_hdg_tol_;
}

PathFollower::TrackingErrors PathFollower::computePathTrackingErrors(
    const geometry_msgs::msg::Pose& robot_pose,
    double robot_yaw,
    const geometry_msgs::msg::PoseStamped& nearest_path_pose,
    bool forward)
{
    const double path_yaw = yawFromPose(nearest_path_pose.pose);
    const double path_eff = forward ? path_yaw : wrapToPi(path_yaw + M_PI);
    const double robot_eff = forward ? robot_yaw : wrapToPi(robot_yaw + M_PI);
    const double dx = robot_pose.position.x - nearest_path_pose.pose.position.x;
    const double dy = robot_pose.position.y - nearest_path_pose.pose.position.y;
    return {
        -std::sin(path_eff) * dx + std::cos(path_eff) * dy,
        wrapToPi(path_eff - robot_eff),
        std::hypot(dx, dy)
    };
}

bool PathFollower::trackingErrorExceeded(const TrackingErrors& e) const
{
    return std::abs(e.lateral_m)  > max_lateral_error_m_
        || std::abs(e.heading_rad) > max_heading_error_rad_
        || e.distance_m             > max_target_distance_m_;
}

double PathFollower::updateAdaptiveBias(
    double lateral_error, double dt, double prev_bias) const
{
    if (!use_adaptive_bias_) return 0.0;
    double err = std::abs(lateral_error) > adaptive_kappa_deadband_ ? lateral_error : 0.0;
    double decayed = prev_bias * std::max(0.0, 1.0 - adaptive_kappa_decay_ * std::max(dt, 0.0));
    double updated = decayed + adaptive_kappa_i_gain_ * err * std::max(dt, 0.0);
    return mmClamp(updated, -adaptive_kappa_limit_, adaptive_kappa_limit_);
}

// ---------------------------------------------------------------------------
// Publish helpers
// ---------------------------------------------------------------------------
void PathFollower::publishZero()
{
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = now();
    cmd_pub_->publish(zero);
    if (use_articulation_servo_ && !external_command_mux_ && articulation_pub_) {
        std_msgs::msg::Float64 center;
        center.data = 0.0;
        articulation_pub_->publish(center);
    }
    if (use_speed_servo_ && !external_command_mux_ && speed_setpoint_pub_) {
        std_msgs::msg::Float64 stop;
        stop.data = 0.0;
        speed_setpoint_pub_->publish(stop);
    }
}

void PathFollower::startArticulationRecenter()
{
    const bool articulation_command_available =
        use_articulation_servo_ || external_command_mux_;
    if (!articulation_command_available || articulation_recenter_s_ <= 0.0) {
        recenter_active_ = false;
        return;
    }
    recenter_active_ = true;
    recenter_until_ = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(articulation_recenter_s_));
}

void PathFollower::requestReplayCancel()
{
    if (!wiln_command_pub_) return;
    std_msgs::msg::String cancel;
    cancel.data = "cancel";
    wiln_command_pub_->publish(cancel);
}

void PathFollower::publishCommand(double linear_x, double steer_norm, double psi_cmd)
{
    if (use_articulation_servo_ && !external_command_mux_ && articulation_pub_) {
        std_msgs::msg::Float64 setpt;
        setpt.data = psi_cmd;
        articulation_pub_->publish(setpt);
    }
    if (use_speed_servo_ && !external_command_mux_ && speed_setpoint_pub_) {
        std_msgs::msg::Float64 spd;
        spd.data = std::abs(linear_x);
        speed_setpoint_pub_->publish(spd);
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp   = now();
    cmd.twist.linear.x = linear_x;
    cmd.twist.angular.z =
        (external_command_mux_ || !use_articulation_servo_) ? steer_norm : 0.0;
    cmd_pub_->publish(cmd);
}

void PathFollower::publishDebug(
    double lat, double hdg, double dist,
    double kd, double kff, double kb,
    double kc, double ke, double slip,
    double psi_raw, double psi_cmd, double steer)
{
    auto pub = [this](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& p, double v) {
        std_msgs::msg::Float64 msg; msg.data = v; p->publish(msg);
    };
    pub(debug_lateral_pub_,      lat);
    pub(debug_heading_pub_,      hdg);
    pub(debug_target_dist_pub_,  dist);
    pub(debug_kappa_desired_pub_,kd);
    pub(debug_kappa_ff_pub_,     kff);
    pub(debug_kappa_bias_pub_,   kb);
    pub(debug_kappa_cmd_pub_,    kc);
    pub(debug_kappa_eff_pub_,    ke);
    pub(debug_slip_pub_,         slip);
    pub(debug_psi_raw_pub_,      psi_raw);
    pub(debug_psi_cmd_pub_,      psi_cmd);
    pub(debug_steering_pub_,     steer);
}

void PathFollower::publishFollowerState(uint8_t state_code, const std::string& detail)
{
    wiln::msg::WilnState msg;
    msg.stamp  = now();
    msg.state  = state_code;
    msg.detail = detail;
    follower_state_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Fallback dead-reckoning
// ---------------------------------------------------------------------------
geometry_msgs::msg::Pose PathFollower::propagateIcpPose(
    const geometry_msgs::msg::Pose& icp_ref,
    const geometry_msgs::msg::Pose& fb_ref,
    const geometry_msgs::msg::Pose& fb_now)
{
    // Encoder deltas in encoder-odom frame
    const double dx_fb   = fb_now.position.x - fb_ref.position.x;
    const double dy_fb   = fb_now.position.y - fb_ref.position.y;
    const double yaw_fb0 = yawFromPose(fb_ref);
    const double yaw_icp0= yawFromPose(icp_ref);
    const double dyaw    = yawFromPose(fb_now) - yaw_fb0;

    // Rotate encoder delta into map frame using heading difference at switch time
    const double rot   = yaw_icp0 - yaw_fb0;
    const double dx_map = dx_fb * std::cos(rot) - dy_fb * std::sin(rot);
    const double dy_map = dx_fb * std::sin(rot) + dy_fb * std::cos(rot);

    const double yaw_est = yaw_icp0 + dyaw;
    const double cy = std::cos(yaw_est * 0.5);
    const double sy = std::sin(yaw_est * 0.5);

    geometry_msgs::msg::Pose est;
    est.position.x    = icp_ref.position.x + dx_map;
    est.position.y    = icp_ref.position.y + dy_map;
    est.position.z    = icp_ref.position.z;
    est.orientation.w = cy;
    est.orientation.x = 0.0;
    est.orientation.y = 0.0;
    est.orientation.z = sy;
    return est;
}

double PathFollower::yawFromPose(const geometry_msgs::msg::Pose& pose)
{
    const auto& q = pose.orientation;
    return std::atan2(2.0*(q.w*q.z + q.x*q.y),
                      q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
}

double PathFollower::distXY(const geometry_msgs::msg::Pose& a,
                             const geometry_msgs::msg::Pose& b)
{
    return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

} // namespace wiln

// ---------------------------------------------------------------------------
// main -- MultiThreadedExecutor (control loop on dedicated thread)
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wiln::PathFollower>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
