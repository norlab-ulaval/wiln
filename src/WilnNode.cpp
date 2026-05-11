#include "wiln/WilnNode.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <filesystem>
#include <cmath>

#include "wiln/ObstacleManager.hpp"
#include "wiln/RobotModel.hpp"
#include "wiln/TrajectoryManager.hpp"

namespace wiln {

// Quaternion that rotates a pose by 180° around Z (reverses direction).
static const tf2::Quaternion HALF_TURN_ROTATION(0.0, 0.0, 1.0, 0.0);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WilnNode::WilnNode() : Node("wiln_node") {
    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    trajectory_speed_         = this->declare_parameter("trajectory_speed", 0.3);
    int smoothing_window      = this->declare_parameter("smoothing_window_size", 15);
    double kappa_max          = this->declare_parameter("kappa_max", 0.7);
    std::string robot_type    = this->declare_parameter("robot_model", std::string("generic"));
    // Patch 1: articulation limit — MUST match the path follower's psi_max_rad.
    // Default 0.785 rad (45°) is safe/conservative.  Set to 1.047 rad (60°) for MTT.
    double articulation_limit_rad = this->declare_parameter("articulation_limit_rad", 0.785);

    // Topic for the live control plan consumed by mtt_path_follower (Patch 2).
    const std::string control_local_plan_topic =
        this->declare_parameter("control_local_plan_topic",
                                std::string("/wiln/control/local_plan"));

    // Obstacle manager parameters
    std::vector<std::string> lidar_topics = this->declare_parameter<std::vector<std::string>>(
        "lidar_topics", std::vector<std::string>{"/merged_points_filtered"});
    std::string target_frame  = this->declare_parameter("target_frame", std::string("map"));
    double obs_max_age        = this->declare_parameter("obstacle_max_age_s", 0.5);
    double crop_length        = this->declare_parameter("crop_length", 25.0);
    double crop_width         = this->declare_parameter("crop_width", 8.0);
    double crop_z_min         = this->declare_parameter("crop_z_min", -0.3);
    double crop_z_max         = this->declare_parameter("crop_z_max",  2.0);
    double voxel_size         = this->declare_parameter("voxel_size", 0.15);
    double min_range          = this->declare_parameter("min_range", 0.5);
    double max_range          = this->declare_parameter("max_range", 20.0);

    // PathDeformer parameters
    double max_deformation_step  = this->declare_parameter("max_deformation_step", 0.25);
    double max_total_deformation = this->declare_parameter("max_total_deformation", 1.0);
    double time_budget_ms        = this->declare_parameter("time_budget_ms", 8.0);

    // TrajectoryStreamer parameters
    int search_window_bw = this->declare_parameter("search_window_backward", 10);
    int search_window_fw = this->declare_parameter("search_window_forward",  80);

    // Service / action topic names — match norlab_robot config/_wiln.yaml
    const std::string follow_path_topic =
        this->declare_parameter("follow_path_topic", std::string("/follow_path"));
    const std::string enable_mapping_service =
        this->declare_parameter("enable_mapping_service", std::string("/mapping/enable_mapping"));
    const std::string disable_mapping_service =
        this->declare_parameter("disable_mapping_service", std::string("/mapping/disable_mapping"));
    const std::string save_map_service =
        this->declare_parameter("save_map_service", std::string("/mapping/save_map"));
    const std::string load_map_service =
        this->declare_parameter("load_map_service", std::string("/mapping/load_map"));

    // ----------------------------------------------------------------
    // Callback groups
    // ----------------------------------------------------------------
    services_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stream_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ----------------------------------------------------------------
    // Core components
    // ----------------------------------------------------------------
    TeachRecorder::Params teach_params;
    teach_params.smoothing_window = smoothing_window;
    recorder_ = std::make_unique<TeachRecorder>(teach_params);

    TrajectoryStreamer::Params stream_params;
    stream_params.search_window_backward = static_cast<size_t>(search_window_bw);
    stream_params.search_window_forward  = static_cast<size_t>(search_window_fw);
    streamer_ = std::make_unique<TrajectoryStreamer>(stream_params);

    PathDeformer::Params deform_params;
    deform_params.max_deformation_step  = max_deformation_step;
    deform_params.max_total_deformation = max_total_deformation;
    deform_params.time_budget_ms        = time_budget_ms;
    deformer_ = std::make_unique<PathDeformer>(deform_params);

    // Robot model — owns kappa_max and articulation limits.
    // PathDeformer reads kappaMax() via setRobotModel().
    robot_model_ = makeRobotModel(robot_type, kappa_max, articulation_limit_rad);
    deformer_->setRobotModel(robot_model_.get());

    // ----------------------------------------------------------------
    // TF2
    // ----------------------------------------------------------------
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ----------------------------------------------------------------
    // Obstacle manager (multi-LiDAR)
    // ----------------------------------------------------------------
    ObstacleParams obs_params;
    obs_params.lidar_topics      = lidar_topics;
    obs_params.target_frame      = target_frame;
    obs_params.obstacle_max_age_s = obs_max_age;
    obs_params.crop_length       = crop_length;
    obs_params.crop_width        = crop_width;
    obs_params.crop_z_min        = crop_z_min;
    obs_params.crop_z_max        = crop_z_max;
    obs_params.voxel_size        = voxel_size;
    obs_params.min_range         = min_range;
    obs_params.max_range         = max_range;
    obstacles_ = std::make_unique<ObstacleManager>(this, obs_params, tf_buffer_, stream_group_);

    // ----------------------------------------------------------------
    // Publishers
    // ----------------------------------------------------------------
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    auto rt_qos        = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    // Transient-local: full recorded/loaded route for rviz/Foxglove
    global_plan_pub_   = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory",  transient_qos);

    // *** REAL CONTROL TOPIC — mtt_path_follower subscribes to this for live steering ***
    // Topic: parametrised via control_local_plan_topic (default /wiln/control/local_plan).
    local_plan_pub_    = this->create_publisher<nav_msgs::msg::Path>(control_local_plan_topic, rt_qos);

    // Debug / Foxglove visualisation — NOT consumed by the controller
    horizon_debug_pub_ = this->create_publisher<nav_msgs::msg::Path>("wiln/debug/horizon",       rt_qos);
    deformed_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("wiln/debug/deformed_plan", rt_qos);

    markers_pub_       = this->create_publisher<visualization_msgs::msg::MarkerArray>("wiln/markers", rt_qos);
    status_pub_        = this->create_publisher<std_msgs::msg::String>("wiln/status",          rt_qos);

    // ----------------------------------------------------------------
    // Subscriptions (stream group — hot path)
    // ----------------------------------------------------------------
    rclcpp::SubscriptionOptions stream_opts;
    stream_opts.callback_group = stream_group_;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_in", rclcpp::QoS(10).best_effort(),
        std::bind(&WilnNode::onOdom, this, std::placeholders::_1), stream_opts);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose_in", rclcpp::QoS(10).best_effort(),
        std::bind(&WilnNode::onPose, this, std::placeholders::_1), stream_opts);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel_in", rclcpp::QoS(10).best_effort(),
        std::bind(&WilnNode::onCmdVel, this, std::placeholders::_1), stream_opts);

    articulation_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/mtt/articulation_angle", rclcpp::QoS(10).best_effort(),
        std::bind(&WilnNode::onArticulationAngle, this, std::placeholders::_1), stream_opts);

    // ----------------------------------------------------------------
    // Services (services group — non-blocking for stream timer)
    // ----------------------------------------------------------------
    start_recording_srv_ = this->create_service<std_srvs::srv::Empty>(
        "start_recording",
        std::bind(&WilnNode::handleStartRecording, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    stop_recording_srv_ = this->create_service<std_srvs::srv::Empty>(
        "stop_recording",
        std::bind(&WilnNode::handleStopRecording, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    play_line_srv_ = this->create_service<std_srvs::srv::Empty>(
        "play_line",
        std::bind(&WilnNode::handlePlayLine, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    cancel_srv_ = this->create_service<std_srvs::srv::Empty>(
        "cancel_trajectory",
        std::bind(&WilnNode::handleCancel, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    clear_trajectory_srv_ = this->create_service<std_srvs::srv::Empty>(
        "clear_trajectory",
        std::bind(&WilnNode::handleClearTrajectory, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    smooth_trajectory_srv_ = this->create_service<std_srvs::srv::Empty>(
        "smooth_trajectory",
        std::bind(&WilnNode::handleSmoothTrajectory, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    save_ltr_srv_ = this->create_service<wiln::srv::SaveMapTraj>(
        "save_map_traj",
        std::bind(&WilnNode::handleSaveLTR, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    load_ltr_srv_ = this->create_service<wiln::srv::LoadMapTraj>(
        "load_map_traj",
        std::bind(&WilnNode::handleLoadLTR, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), services_group_);

    // ----------------------------------------------------------------
    // Action / service clients
    // ----------------------------------------------------------------
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, follow_path_topic);

    enable_mapping_client_  = this->create_client<std_srvs::srv::Empty>(
        enable_mapping_service,  rclcpp::ServicesQoS(), services_group_);
    disable_mapping_client_ = this->create_client<std_srvs::srv::Empty>(
        disable_mapping_service, rclcpp::ServicesQoS(), services_group_);
    save_map_client_ = this->create_client<SaveMap>(
        save_map_service, rclcpp::ServicesQoS(), services_group_);
    load_map_client_ = this->create_client<LoadMap>(
        load_map_service, rclcpp::ServicesQoS(), services_group_);

    // ----------------------------------------------------------------
    // 10 Hz stream timer
    // ----------------------------------------------------------------
    stream_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&WilnNode::streamLoop, this),
        stream_group_);

    RCLCPP_INFO(get_logger(),
        "WILN ready — model=%s  kappa_max_nominal=%.2f 1/m  "
        "articulation_limit=%.3f rad (%.1f°)  speed=%.2f m/s",
        robot_model_->name().c_str(),
        kappa_max,
        articulation_limit_rad,
        articulation_limit_rad * 180.0 / M_PI,
        trajectory_speed_);
    RCLCPP_INFO(get_logger(),
        "  control_plan -> %s   debug_horizon -> wiln/debug/horizon   "
        "debug_deformed -> wiln/debug/deformed_plan",
        control_local_plan_topic.c_str());
}

// ---------------------------------------------------------------------------
// Subscriptions — hot path
// ---------------------------------------------------------------------------
void WilnNode::onOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
    auto ptr = std::make_shared<const geometry_msgs::msg::Pose>(msg->pose.pose);
    std::lock_guard<std::mutex> lock(pose_ptr_mutex_);
    current_pose_ptr_ = std::move(ptr);
}

void WilnNode::onPose(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (state_.load() == WilnState::RECORDING) {
        recorder_->addPose(*msg, driving_forward_.load());
    }
}

void WilnNode::onCmdVel(geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    driving_forward_.store(msg->twist.linear.x >= 0.0);
}

void WilnNode::onArticulationAngle(std_msgs::msg::Float64::SharedPtr msg) {
    // MttModel is the only built-in model that reacts to articulation angle.
    // GenericModel ignores it (static kappa_max).
    if (auto* mtt = dynamic_cast<MttModel*>(robot_model_.get())) {
        mtt->updateArticulationAngle(msg->data);
    }
}

// ---------------------------------------------------------------------------
// Stream loop — 10 Hz
// ---------------------------------------------------------------------------
void WilnNode::streamLoop() {
    if (state_.load() != WilnState::PLAYING) return;

    // --- Snapshot current pose (pointer swap, never blocks) ---
    std::shared_ptr<const geometry_msgs::msg::Pose> pose_ptr;
    {
        std::lock_guard<std::mutex> lock(pose_ptr_mutex_);
        pose_ptr = current_pose_ptr_;
    }
    if (!pose_ptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "streamLoop: no pose available — skipping tick.");
        return;
    }

    // --- Local horizon ---
    TrajectoryStreamer::Diag stream_diag;
    auto local_horizon = streamer_->getLocalHorizon(*pose_ptr, &stream_diag);
    if (local_horizon.poses.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "streamLoop: empty local horizon — trajectory finished?");
        return;
    }

    // --- Obstacle snapshot (never blocks — reads cached data) ---
    ObstacleManager::Diag obs_diag;
    auto obs_snapshot = obstacles_->getSnapshot(*pose_ptr, &obs_diag);
    const std::vector<Eigen::Vector3d> empty_obs;
    const auto& obstacles = obs_snapshot ? *obs_snapshot : empty_obs;

    // --- Deform (time-budgeted; falls back to original horizon if over budget) ---
    PathDeformer::Diag deform_diag;
    auto safe_plan = deformer_->deform(local_horizon, obstacles, &deform_diag);
    safe_plan.header.stamp = this->now();

    if (deform_diag.used_fallback) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "PathDeformer fallback (budget=%.1f ms used, time_exceeded=%s, "
            "dist_to_path=%.2f m)",
            deform_diag.deform_time_ms,
            deform_diag.time_budget_exceeded ? "yes" : "no",
            stream_diag.dist_to_path_m);
    }

    // --- Fallback cache: keep last valid deformed plan ---
    bool plan_is_valid = !deform_diag.used_fallback && safe_plan.poses.size() >= 3;
    if (plan_is_valid) {
        std::lock_guard<std::mutex> lk(last_valid_plan_mutex_);
        last_valid_plan_ = safe_plan;
    } else {
        std::lock_guard<std::mutex> lk(last_valid_plan_mutex_);
        if (!last_valid_plan_.poses.empty()) {
            safe_plan = last_valid_plan_;
        }
    }

    // --- Publish control plan ---
    // safe_plan is the obstacle-deformed horizon (or the last valid plan on fallback).
    // This IS the live steering reference for mtt_path_follower — not diagnostic-only.
    local_plan_pub_->publish(safe_plan);

    // --- Publish debug topics (Foxglove only — NOT consumed by the controller) ---
    horizon_debug_pub_->publish(local_horizon);   // raw undeformed horizon
    deformed_plan_pub_->publish(safe_plan);        // elastic-band result (same as control when valid)

    // --- Status ---
    {
        std_msgs::msg::String status_msg;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "state=%s dist=%.2fm heading=%.1fdeg deform=%.1fms obs=%zu",
            toString(state_.load()),
            stream_diag.dist_to_path_m,
            stream_diag.heading_error_rad * 180.0 / M_PI,
            deform_diag.deform_time_ms,
            obstacles.size());
        status_msg.data = buf;
        status_pub_->publish(status_msg);
    }

    publishVisualization(local_horizon, safe_plan, obstacles, stream_diag, deform_diag);
}

// ---------------------------------------------------------------------------
// Service handlers
// ---------------------------------------------------------------------------
void WilnNode::handleStartRecording(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    WilnState expected = WilnState::IDLE;
    if (!state_.compare_exchange_strong(expected, WilnState::RECORDING)) {
        RCLCPP_WARN(get_logger(), "Cannot start recording from state %s", toString(state_.load()));
        return;
    }
    recorder_->start();
    RCLCPP_INFO(get_logger(), "Recording started.");
}

void WilnNode::handleStopRecording(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    if (state_.load() != WilnState::RECORDING) {
        RCLCPP_WARN(get_logger(), "Not recording — ignoring stop_recording.");
        return;
    }
    recorder_->stop();
    recorder_->smooth();
    state_.store(WilnState::IDLE);

    auto traj = recorder_->getTrajectory();
    size_t total = 0;
    for (const auto& p : traj.paths) total += p.poses.size();
    RCLCPP_INFO(get_logger(), "Recording stopped and smoothed: %zu segments, %zu poses total.",
        traj.paths.size(), total);

    publishGlobalPlan(traj);
}

void WilnNode::handlePlayLine(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    WilnState expected = WilnState::IDLE;
    if (!state_.compare_exchange_strong(expected, WilnState::PLAYING)) {
        RCLCPP_WARN(get_logger(), "Cannot play from state %s", toString(state_.load()));
        return;
    }

    auto traj = recorder_->getTrajectory();
    if (traj.paths.empty() || traj.paths.front().poses.empty()) {
        RCLCPP_WARN(get_logger(), "No trajectory to play.");
        state_.store(WilnState::IDLE);
        return;
    }

    // Determine start/end proximity to choose direction
    std::shared_ptr<const geometry_msgs::msg::Pose> pose_ptr;
    {
        std::lock_guard<std::mutex> lock(pose_ptr_mutex_);
        pose_ptr = current_pose_ptr_;
    }
    if (!pose_ptr) {
        RCLCPP_WARN(get_logger(), "No pose available — cannot determine start/end proximity.");
        state_.store(WilnState::IDLE);
        return;
    }

    const auto& first_pose = traj.paths.front().poses.front();
    const auto& last_pose  = traj.paths.back().poses.back();
    double dist_to_start   = distanceToPose(*pose_ptr, first_pose);
    double dist_to_end     = distanceToPose(*pose_ptr, last_pose);

    RCLCPP_INFO(get_logger(), "Distance to start: %.2f m  |  Distance to end: %.2f m",
        dist_to_start, dist_to_end);

    if (dist_to_end < dist_to_start) {
        RCLCPP_INFO(get_logger(), "Closer to end — reversing trajectory.");
        traj = reverseTrajectory(traj);
    }

    streamer_->setBaseTrajectory(traj);

    // Disable mapping during repeat
    if (disable_mapping_client_->service_is_ready()) {
        disable_mapping_client_->async_send_request(
            std::make_shared<std_srvs::srv::Empty::Request>());
    }

    // Send FollowPath action goal
    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "/follow_path action server unavailable.");
        state_.store(WilnState::IDLE);
        return;
    }

    norlab_controllers_msgs::msg::FollowerOptions opts;
    opts.init_mode.data = 1;  // 1 = no brake, start from closest point
    opts.velocity.data  = static_cast<float>(trajectory_speed_);

    FollowPath::Goal goal;
    goal.follower_options = opts;
    goal.path             = traj;

    auto send_opts = rclcpp_action::Client<FollowPath>::SendGoalOptions{};
    send_opts.goal_response_callback = [this](GoalHandle::SharedPtr gh) {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        if (!gh) {
            RCLCPP_ERROR(get_logger(), "FollowPath goal rejected by server.");
            state_.store(WilnState::IDLE);
            return;
        }
        active_goal_handle_ = gh;
        RCLCPP_INFO(get_logger(), "FollowPath goal accepted.");
    };
    send_opts.result_callback = [this](const GoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        active_goal_handle_.reset();
        state_.store(WilnState::IDLE);
        if (enable_mapping_client_->service_is_ready()) {
            enable_mapping_client_->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>());
        }
        RCLCPP_INFO(get_logger(), "FollowPath finished with status %d.",
            static_cast<int>(result.code));
    };

    follow_path_client_->async_send_goal(goal, send_opts);
    RCLCPP_INFO(get_logger(), "Playback started at %.2f m/s.", trajectory_speed_);
}

void WilnNode::handleCancel(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    state_.store(WilnState::IDLE);
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        if (active_goal_handle_) {
            follow_path_client_->async_cancel_goal(active_goal_handle_);
            active_goal_handle_.reset();
        }
    }
    if (enable_mapping_client_->service_is_ready()) {
        enable_mapping_client_->async_send_request(
            std::make_shared<std_srvs::srv::Empty::Request>());
    }
    RCLCPP_INFO(get_logger(), "Trajectory cancelled.");
}

void WilnNode::handleClearTrajectory(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    if (state_.load() == WilnState::PLAYING) {
        RCLCPP_WARN(get_logger(), "Cannot clear trajectory while playing.");
        return;
    }
    recorder_->clear();
    RCLCPP_INFO(get_logger(), "Trajectory cleared.");
}

void WilnNode::handleSmoothTrajectory(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    WilnState s = state_.load();
    if (s == WilnState::RECORDING || s == WilnState::PLAYING) {
        RCLCPP_WARN(get_logger(), "Cannot smooth during recording/playing.");
        return;
    }
    recorder_->smooth();
    publishGlobalPlan(recorder_->getTrajectory());
    RCLCPP_INFO(get_logger(), "Trajectory smoothed.");
}

void WilnNode::handleSaveLTR(
    std::shared_ptr<wiln::srv::SaveMapTraj::Request> req,
    std::shared_ptr<wiln::srv::SaveMapTraj::Response>)
{
    WilnState expected = WilnState::IDLE;
    if (!state_.compare_exchange_strong(expected, WilnState::SAVING)) {
        RCLCPP_WARN(get_logger(), "Cannot save from state %s", toString(state_.load()));
        return;
    }

    auto traj = recorder_->getTrajectory();
    if (traj.paths.empty()) {
        RCLCPP_WARN(get_logger(), "No trajectory to save.");
        state_.store(WilnState::IDLE);
        return;
    }

    const std::string map_path = req->file_name.data + ".vtk";
    bool ok = TrajectoryManager::saveLTR(req->file_name.data, traj);
    state_.store(WilnState::IDLE);

    if (ok) {
        RCLCPP_INFO(get_logger(), "Trajectory saved to %s", req->file_name.data.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to save trajectory to %s", req->file_name.data.c_str());
    }

    // Save VTK map best-effort. Do not spin this node inside its own executor
    // callback: that crashes with "node has already been added to an executor".
    if (save_map_client_->service_is_ready()) {
        auto map_req = std::make_shared<SaveMap::Request>();
        map_req->map_file_name.data = map_path;
        save_map_client_->async_send_request(
            map_req,
            [this, map_path](rclcpp::Client<SaveMap>::SharedFuture) {
                RCLCPP_INFO(get_logger(), "Map save requested to %s", map_path.c_str());
            });
    } else {
        RCLCPP_WARN(get_logger(), "Mapping save service unavailable — saved trajectory only.");
    }
}

void WilnNode::handleLoadLTR(
    std::shared_ptr<wiln::srv::LoadMapTraj::Request> req,
    std::shared_ptr<wiln::srv::LoadMapTraj::Response>)
{
    WilnState expected = WilnState::IDLE;
    if (!state_.compare_exchange_strong(expected, WilnState::LOADING)) {
        RCLCPP_WARN(get_logger(), "Cannot load from state %s", toString(state_.load()));
        return;
    }

    norlab_controllers_msgs::msg::PathSequence traj;
    if (!TrajectoryManager::loadLTR(req->file_name.data, traj)) {
        RCLCPP_ERROR(get_logger(), "Failed to load LTR from %s", req->file_name.data.c_str());
        state_.store(WilnState::IDLE);
        return;
    }

    recorder_->setTrajectory(traj);
    streamer_->setBaseTrajectory(traj);
    state_.store(WilnState::IDLE);

    size_t total = 0;
    for (const auto& p : traj.paths) total += p.poses.size();
    RCLCPP_INFO(get_logger(), "Trajectory loaded: %zu segments, %zu poses.",
        traj.paths.size(), total);

    publishGlobalPlan(traj);

    // Load VTK map best-effort after the route is visible. Do not block/spin
    // from the service callback; WILN is already owned by the executor.
    std::string vtk_path = req->file_name.data + ".vtk";
    if (load_map_client_->service_is_ready() && std::filesystem::exists(vtk_path)) {
        auto map_req = std::make_shared<LoadMap::Request>();
        map_req->map_file_name.data = vtk_path;
        if (!traj.paths.empty() && !traj.paths.front().poses.empty()) {
            map_req->pose = traj.paths.front().poses.front().pose;
        }
        load_map_client_->async_send_request(
            map_req,
            [this, vtk_path](rclcpp::Client<LoadMap>::SharedFuture) {
                RCLCPP_INFO(get_logger(), "Map load requested from %s", vtk_path.c_str());
            });
    } else if (!std::filesystem::exists(vtk_path)) {
        RCLCPP_WARN(get_logger(), "No VTK map next to route (%s) — loaded trajectory only.",
            vtk_path.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "Mapping load service unavailable — loaded trajectory only.");
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void WilnNode::publishGlobalPlan(const norlab_controllers_msgs::msg::PathSequence& traj) {
    nav_msgs::msg::Path global;
    global.header = traj.header;
    global.header.stamp = this->now();
    for (const auto& path : traj.paths) {
        for (const auto& p : path.poses) {
            global.poses.push_back(p);
        }
    }
    global_plan_pub_->publish(global);
}

void WilnNode::publishVisualization(
    const nav_msgs::msg::Path&          horizon,
    const nav_msgs::msg::Path&          deformed,
    const std::vector<Eigen::Vector3d>& obstacles,
    const TrajectoryStreamer::Diag&     stream_diag,
    const PathDeformer::Diag&           deform_diag)
{
    if (markers_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray array;
    auto now = this->now();
    const std::string frame = deformed.header.frame_id.empty() ? "map" : deformed.header.frame_id;

    auto make_strip = [&](int id, float r, float g, float b,
                          const nav_msgs::msg::Path& path) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp    = now;
        m.ns              = "wiln";
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.scale.x         = 0.08;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9f;
        m.lifetime        = rclcpp::Duration::from_seconds(0.5);
        for (const auto& ps : path.poses) {
            geometry_msgs::msg::Point pt;
            pt.x = ps.pose.position.x;
            pt.y = ps.pose.position.y;
            pt.z = ps.pose.position.z;
            m.points.push_back(pt);
        }
        return m;
    };

    // Marker 0: local horizon (cyan)
    array.markers.push_back(make_strip(0, 0.0f, 1.0f, 1.0f, horizon));
    // Marker 1: deformed plan (yellow — DIAGNOSTIC ONLY, not consumed by controller)
    array.markers.push_back(make_strip(1, 1.0f, 1.0f, 0.0f, deformed));

    // Marker 2: obstacles in repulsion radius (red cubes, colour-coded by displacement contribution)
    {
        visualization_msgs::msg::Marker obs_m;
        obs_m.header.frame_id = frame;
        obs_m.header.stamp    = now;
        obs_m.ns              = "wiln";
        obs_m.id              = 2;
        obs_m.type            = visualization_msgs::msg::Marker::CUBE_LIST;
        obs_m.action          = visualization_msgs::msg::Marker::ADD;
        obs_m.scale.x = obs_m.scale.y = obs_m.scale.z = 0.12;
        obs_m.lifetime = rclcpp::Duration::from_seconds(0.5);

        const double rep_dist = 1.5;  // keep consistent with PathDeformer default
        for (const auto& o : obstacles) {
            // Only show points near the deformed path (coarse range filter)
            bool near_path = false;
            for (size_t k = 0; k < deformed.poses.size(); k += 5) {
                const auto& pp = deformed.poses[k].pose.position;
                double d2 = (o.x()-pp.x)*(o.x()-pp.x) + (o.y()-pp.y)*(o.y()-pp.y);
                if (d2 < (rep_dist * 3.0) * (rep_dist * 3.0)) { near_path = true; break; }
            }
            if (!near_path) continue;

            geometry_msgs::msg::Point pt;
            pt.x = o.x(); pt.y = o.y(); pt.z = o.z();
            obs_m.points.push_back(pt);

            // Colour: bright red near repulsion onset, fade to orange further away
            std_msgs::msg::ColorRGBA c;
            c.r = 1.0f; c.g = 0.3f; c.b = 0.1f; c.a = 0.75f;
            obs_m.colors.push_back(c);
        }
        array.markers.push_back(obs_m);
    }

    // Marker 3: robot footprint polygon (from RobotModel::sweptArea)
    if (robot_model_) {
        auto area = robot_model_->sweptArea();
        if (!area.empty()) {
            visualization_msgs::msg::Marker footprint;
            footprint.header.frame_id = frame;
            footprint.header.stamp    = now;
            footprint.ns              = "wiln";
            footprint.id              = 3;
            footprint.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            footprint.action          = visualization_msgs::msg::Marker::ADD;
            footprint.scale.x         = 0.05;
            footprint.color.r = 0.0f; footprint.color.g = 0.8f;
            footprint.color.b = 1.0f; footprint.color.a = 0.6f;
            footprint.lifetime = rclcpp::Duration::from_seconds(0.5);

            // Find current robot pose for footprint anchor
            std::shared_ptr<const geometry_msgs::msg::Pose> pose_ptr;
            {
                std::lock_guard<std::mutex> lock(pose_ptr_mutex_);
                pose_ptr = current_pose_ptr_;
            }
            if (pose_ptr) {
                const auto& q = pose_ptr->orientation;
                double yaw = std::atan2(
                    2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
                double ca = std::cos(yaw), sa = std::sin(yaw);
                double rx = pose_ptr->position.x, ry = pose_ptr->position.y;
                double rz = pose_ptr->position.z;

                for (const auto& v : area) {
                    geometry_msgs::msg::Point pt;
                    pt.x = rx + ca * v.x() - sa * v.y();
                    pt.y = ry + sa * v.x() + ca * v.y();
                    pt.z = rz;
                    footprint.points.push_back(pt);
                }
                // Close the polygon
                if (!footprint.points.empty())
                    footprint.points.push_back(footprint.points.front());
                array.markers.push_back(footprint);
            }
        }
    }

    // Marker 4: timing + status text (above midpoint of deformed path)
    {
        visualization_msgs::msg::Marker text_m;
        text_m.header.frame_id = frame;
        text_m.header.stamp    = now;
        text_m.ns              = "wiln";
        text_m.id              = 4;
        text_m.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_m.action          = visualization_msgs::msg::Marker::ADD;
        text_m.scale.z         = 0.35;
        text_m.color.r = text_m.color.g = text_m.color.b = text_m.color.a = 1.0f;
        text_m.lifetime = rclcpp::Duration::from_seconds(1.0);

        if (!deformed.poses.empty()) {
            auto& p = deformed.poses[deformed.poses.size() / 2].pose.position;
            text_m.pose.position.x = p.x;
            text_m.pose.position.y = p.y;
            text_m.pose.position.z = p.z + 0.7;
        }

        char buf[192];
        std::snprintf(buf, sizeof(buf),
            "WILN:%s  model:%s\n"
            "dist:%.2fm  hdg:%.1f°  idx:%zu\n"
            "deform:%.1fms  obs:%zu  disp:%.2fm%s",
            toString(state_.load()),
            robot_model_ ? robot_model_->name().c_str() : "?",
            stream_diag.dist_to_path_m,
            stream_diag.heading_error_rad * 180.0 / M_PI,
            stream_diag.closest_global_idx,
            deform_diag.deform_time_ms,
            deform_diag.obstacle_count,
            deform_diag.max_displacement_m,
            deform_diag.used_fallback ? " [FALLBACK]" : "");
        text_m.text = buf;
        array.markers.push_back(text_m);
    }

    // Marker 5: fallback warning (red text if using cached plan)
    if (deform_diag.used_fallback) {
        visualization_msgs::msg::Marker warn;
        warn.header.frame_id = frame;
        warn.header.stamp    = now;
        warn.ns              = "wiln";
        warn.id              = 5;
        warn.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        warn.action          = visualization_msgs::msg::Marker::ADD;
        warn.scale.z         = 0.5;
        warn.color.r = 1.0f; warn.color.g = 0.2f; warn.color.b = 0.0f;
        warn.color.a = 1.0f;
        warn.lifetime = rclcpp::Duration::from_seconds(1.0);
        if (!deformed.poses.empty()) {
            auto& p = deformed.poses.front().pose.position;
            warn.pose.position.x = p.x;
            warn.pose.position.y = p.y + 1.0;
            warn.pose.position.z = p.z + 1.2;
        }
        warn.text = deform_diag.time_budget_exceeded ? "DEFORM TIMEOUT" : "DEFORM FALLBACK";
        array.markers.push_back(warn);
    }

    markers_pub_->publish(array);
}

norlab_controllers_msgs::msg::PathSequence WilnNode::reverseTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& traj) const
{
    norlab_controllers_msgs::msg::PathSequence rev = traj;
    std::reverse(rev.paths.begin(), rev.paths.end());
    for (auto& path : rev.paths) {
        std::reverse(path.poses.begin(), path.poses.end());
        path.forward = !path.forward;
        for (auto& ps : path.poses) {
            tf2::Quaternion q;
            tf2::fromMsg(ps.pose.orientation, q);
            q = HALF_TURN_ROTATION * q;
            q.normalize();
            ps.pose.orientation = tf2::toMsg(q);
        }
    }
    return rev;
}

double WilnNode::distanceToPose(
    const geometry_msgs::msg::Pose&       from,
    const geometry_msgs::msg::PoseStamped& to) const
{
    double dx = from.position.x - to.pose.position.x;
    double dy = from.position.y - to.pose.position.y;
    double dz = from.position.z - to.pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

} // namespace wiln

// ---------------------------------------------------------------------------
// main — MultiThreadedExecutor
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wiln::WilnNode>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
