#include "wiln/PathFollower.hpp"

#include <cmath>
#include <limits>

namespace wiln {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
PathFollower::PathFollower() : Node("wiln_path_follower")
{
    // ----- Parameters -----
    // Topics
    declare_parameter("cmd_vel_topic",      std::string("controller/cmd_vel"));
    declare_parameter("odom_topic",         std::string("/mapping/icp_odom"));
    declare_parameter("local_plan_topic",   std::string("/wiln/control/local_plan"));
    declare_parameter("trajectory_topic",   std::string("/wiln/trajectory"));
    declare_parameter("command_topic",      std::string("/wiln/command"));
    declare_parameter("replay_state_topic", std::string("/wiln/replay/state"));
    declare_parameter("state_topic",        std::string("/wiln/follower/state"));
    declare_parameter("obstacle_stop_topic", std::string("/mtt_obstacle/stop_requested"));
    declare_parameter("obstacle_slowdown_topic", std::string("/mtt_obstacle/slowdown_scale"));
    declare_parameter("control_rate_hz",    20.0);

    // Gains
    k_y_           = declare_parameter("k_y",         0.6);
    k_theta_       = declare_parameter("k_theta",     1.2);
    kappa_max_     = declare_parameter("kappa_max",   0.7);
    advance_dist_  = declare_parameter("advance_distance_m",          0.35);
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
    use_articulation_servo_= declare_parameter("use_articulation_servo", false);
    use_speed_servo_       = declare_parameter("use_speed_servo",        false);

    const std::string cmd_vel_topic    = get_parameter("cmd_vel_topic").as_string();
    const std::string odom_topic       = get_parameter("odom_topic").as_string();
    const std::string local_plan_topic = get_parameter("local_plan_topic").as_string();
    const std::string trajectory_topic = get_parameter("trajectory_topic").as_string();
    const std::string command_topic = get_parameter("command_topic").as_string();
    const std::string replay_state_topic = get_parameter("replay_state_topic").as_string();
    const std::string state_topic = get_parameter("state_topic").as_string();
    const std::string obstacle_stop_topic = get_parameter("obstacle_stop_topic").as_string();
    const std::string obstacle_slowdown_topic = get_parameter("obstacle_slowdown_topic").as_string();
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

    // ----- Publishers -----
    cmd_pub_         = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, rel_qos);
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

    if (use_articulation_servo_)
        articulation_pub_ = create_publisher<std_msgs::msg::Float64>("/mtt_articulation_setpoint", 20);
    if (use_speed_servo_)
        speed_setpoint_pub_ = create_publisher<std_msgs::msg::Float64>("/speed_setpoint", 20);

    // ----- 20 Hz control timer -----
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz));
    control_timer_ = create_wall_timer(period_ns, [this]() { controlLoop(); }, control_group_);

    publishFollowerState(wiln::msg::WilnState::IDLE, "ready");
    RCLCPP_INFO(get_logger(),
        "wiln_path_follower started. odom=%s trajectory=%s command=%s cmd_vel=%s rate=%.0f Hz, speed=%.2f m/s, "
        "local_plan_topic=%s, obstacle_stop=%s, obstacle_slowdown=%s, psi_max=%.1f deg, slip=%s",
        odom_topic.c_str(), trajectory_topic.c_str(), command_topic.c_str(), cmd_vel_topic.c_str(),
        control_rate_hz, default_speed_, local_plan_topic.c_str(),
        obstacle_stop_topic.c_str(), obstacle_slowdown_topic.c_str(),
        motion_params_.max_articulation_rad * 180.0 / M_PI,
        motion_params_.use_slip_heuristic ? "on" : "off");
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------
void PathFollower::onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(odom_mutex_);
    latest_odom_  = *msg;
    odom_stamp_   = now();
    odom_received_= true;
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
    // If replay_node goes IDLE externally (trajectory completed or cancelled),
    // we also stop following.
    if (msg->state == wiln::msg::WilnState::IDLE) {
        std::lock_guard<std::mutex> lk(follow_mutex_);
        if (following_) {
            following_ = false;
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

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------
void PathFollower::handlePlay()
{
    norlab_controllers_msgs::msg::PathSequence traj;
    {
        std::lock_guard<std::mutex> lk(traj_mutex_);
        if (!traj_received_ || cached_trajectory_.paths.empty()) {
            RCLCPP_WARN(get_logger(), "play: no trajectory available.");
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
        return;
    }

    nav_msgs::msg::Odometry odom;
    rclcpp::Time odom_stamp;
    bool odom_received = false;
    {
        std::lock_guard<std::mutex> ok(odom_mutex_);
        odom = latest_odom_;
        odom_stamp = odom_stamp_;
        odom_received = odom_received_;
    }
    if (!odom_received || (now() - odom_stamp).seconds() > odom_timeout_s_) {
        RCLCPP_WARN(get_logger(), "play: no fresh odometry available, refusing to follow.");
        publishZero();
        publishFollowerState(wiln::msg::WilnState::IDLE, "play refused: no fresh odom");
        return;
    }

    // Determine speed from trajectory (fallback to default)
    double speed = default_speed_;

    std::lock_guard<std::mutex> lk(follow_mutex_);
    active_segments_      = std::move(segments);
    active_speed_         = speed;
    current_segment_      = 0;
    following_            = true;
    local_plan_rcvd_once_ = false;
    path_lost_tracking_   = false;
    obstacle_hold_state_published_ = false;
    prev_psi_cmd_         = 0.0;
    kappa_adaptive_bias_  = 0.0;

    waypoint_index_ = findStartIndex(active_segments_[0].poses, odom.pose.pose);

    publishFollowerState(wiln::msg::WilnState::PLAYING, "following");
    RCLCPP_INFO(get_logger(), "Following trajectory (%zu segments, %.2f m/s).",
        active_segments_.size(), active_speed_);
}

void PathFollower::handleCancel()
{
    std::lock_guard<std::mutex> lk(follow_mutex_);
    if (!following_) return;
    following_ = false;
    publishZero();
    publishFollowerState(wiln::msg::WilnState::IDLE, "cancelled");
    RCLCPP_INFO(get_logger(), "Path following cancelled.");
}

void PathFollower::stopFollowing()
{
    // Called from controlLoop — follow_mutex_ is held by caller.
    following_ = false;
    publishZero();
    publishFollowerState(wiln::msg::WilnState::IDLE, "trajectory completed");
}

// ---------------------------------------------------------------------------
// 20 Hz control loop
// ---------------------------------------------------------------------------
void PathFollower::controlLoop()
{
    std::lock_guard<std::mutex> lk(follow_mutex_);
    if (!following_) return;

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

    // --- Get fresh odom ---
    nav_msgs::msg::Odometry odom;
    rclcpp::Time odom_t;
    {
        std::lock_guard<std::mutex> ok(odom_mutex_);
        odom   = latest_odom_;
        odom_t = odom_stamp_;
    }
    if ((now() - odom_t).seconds() > odom_timeout_s_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
            "Odometry stale! Stopping for safety.");
        stopFollowing();
        return;
    }

    const auto& robot_pose = odom.pose.pose;
    const double robot_yaw = yawFromPose(robot_pose);

    auto& segment = active_segments_[current_segment_];
    const bool forward       = segment.forward;
    const bool final_segment = (current_segment_ == static_cast<int>(active_segments_.size()) - 1);

    // --- Advance waypoint on global path ---
    waypoint_index_ = advanceWaypointIndex(segment.poses, waypoint_index_, robot_pose);
    const auto& global_target = segment.poses[waypoint_index_];

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
    std::vector<geometry_msgs::msg::PoseStamped>* active_poses = nullptr;
    int active_index = 0;

    if (local_plan_fresh && !local_plan.poses.empty()) {
        local_plan_rcvd_once_ = true;
        int li = findStartIndex(local_plan.poses, robot_pose);
        li = advanceWaypointIndex(local_plan.poses, li, robot_pose);
        target_pose  = local_plan.poses[li];
        active_poses = &local_plan.poses;
        active_index = li;
    } else if (local_plan_rcvd_once_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Local plan stale — safety stop.");
        publishZero();
        return;
    } else {
        // Fallback to global path before first local plan is received
        target_pose  = global_target;
        active_poses = &segment.poses;
        active_index = waypoint_index_;
    }

    // --- Publish target pose for debug ---
    target_pose_pub_->publish(target_pose);

    // --- Tracking errors ---
    auto errs = computeTrackingErrors(robot_pose, robot_yaw, target_pose, forward);

    if (trackingErrorExceeded(errs)) {
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
            RCLCPP_INFO(get_logger(), "Trajectory completed.");
            stopFollowing();
            return;
        } else {
            current_segment_++;
            waypoint_index_ = findStartIndex(
                active_segments_[current_segment_].poses, robot_pose);
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
    auto ctrl = computeControl(
        robot_pose, robot_yaw, target_pose, forward,
        active_speed_, prev_psi_cmd_,
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

int PathFollower::advanceWaypointIndex(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    int idx, const geometry_msgs::msg::Pose& robot_pose) const
{
    while (idx < static_cast<int>(poses.size()) - 1 &&
           distXY(robot_pose, poses[idx].pose) < advance_dist_)
        ++idx;
    return idx;
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

PathFollower::TrackingErrors PathFollower::computeTrackingErrors(
    const geometry_msgs::msg::Pose& robot_pose, double robot_yaw,
    const geometry_msgs::msg::PoseStamped& target, bool forward)
{
    const double theta_eff = forward ? robot_yaw : wrapToPi(robot_yaw + M_PI);
    const double tgt_yaw   = yawFromPose(target.pose);
    const double tgt_eff   = forward ? tgt_yaw : wrapToPi(tgt_yaw + M_PI);
    const double dx = target.pose.position.x - robot_pose.position.x;
    const double dy = target.pose.position.y - robot_pose.position.y;
    return {
        -std::sin(theta_eff)*dx + std::cos(theta_eff)*dy,
        wrapToPi(tgt_eff - theta_eff),
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
}

void PathFollower::publishCommand(double linear_x, double steer_norm, double psi_cmd)
{
    if (use_articulation_servo_ && articulation_pub_) {
        std_msgs::msg::Float64 setpt;
        setpt.data = psi_cmd;
        articulation_pub_->publish(setpt);
    }
    if (use_speed_servo_ && speed_setpoint_pub_) {
        std_msgs::msg::Float64 spd;
        spd.data = std::abs(linear_x);
        speed_setpoint_pub_->publish(spd);
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp   = now();
    cmd.twist.linear.x = linear_x;
    cmd.twist.angular.z = use_articulation_servo_ ? 0.0 : steer_norm;
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
