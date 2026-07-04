#include "wiln/WilnReplayNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace wiln {

static const tf2::Quaternion HALF_TURN_Z(0.0, 0.0, 1.0, 0.0);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WilnReplayNode::WilnReplayNode() : Node("wiln_replay_node")
{
    // --- Parameters ---
    trajectory_speed_         = declare_parameter("trajectory_speed",          0.40);
    max_start_distance_m_     = declare_parameter("max_start_distance_m",      3.0);
    control_local_plan_topic_ = declare_parameter("control_local_plan_topic",
                                    std::string("/wiln/control/local_plan"));
    const std::string odom_topic = declare_parameter("odom_topic", std::string("odom_in"));
    const std::string obstacles_topic = declare_parameter("obstacles_topic", std::string("/wiln/obstacles"));
    const std::string trajectory_topic = declare_parameter("trajectory_topic", std::string("/wiln/trajectory"));
    const std::string command_topic = declare_parameter("command_topic", std::string("/wiln/command"));
    const std::string articulation_topic = declare_parameter("articulation_topic", std::string("/mtt/articulation_angle"));
    const std::string debug_horizon_topic = declare_parameter("debug_horizon_topic", std::string("/wiln/debug/horizon"));
    const std::string debug_deformed_topic = declare_parameter("debug_deformed_topic", std::string("/wiln/debug/deformed_plan"));
    const std::string markers_topic = declare_parameter("markers_topic", std::string("/wiln/markers"));
    const std::string state_topic = declare_parameter("state_topic", std::string("/wiln/replay/state"));
    const std::string diagnostics_topic = declare_parameter("diagnostics_topic", std::string("/wiln/replay/diagnostics"));
    double kappa_max          = declare_parameter("kappa_max",                 0.70);
    std::string robot_type    = declare_parameter("robot_model",               std::string("generic"));
    double articulation_limit = declare_parameter("articulation_limit_rad",    0.785);

    double max_deformation_step  = declare_parameter("max_deformation_step",  0.25);
    double max_total_deformation = declare_parameter("max_total_deformation", 1.0);
    double time_budget_ms        = declare_parameter("time_budget_ms",         8.0);
    int search_window_bw         = declare_parameter("search_window_backward", 10);
    int search_window_fw         = declare_parameter("search_window_forward",  80);

    const std::string enable_mapping_svc  = declare_parameter(
        "enable_mapping_service",  std::string("/mapping/enable_mapping"));
    const std::string disable_mapping_svc = declare_parameter(
        "disable_mapping_service", std::string("/mapping/disable_mapping"));
    enable_deformation_ = declare_parameter("enable_deformation", true);
    reenable_mapping_on_stop_ = declare_parameter("reenable_mapping_on_stop", true);
    debug_              = declare_parameter("debug",              false);
    if (!enable_deformation_) {
        RCLCPP_INFO(get_logger(),
            "enable_deformation=false: deformer skipped, raw horizon published as control plan.");
    }

    // --- Core components ---
    TrajectoryStreamer::Params stream_params;
    stream_params.search_window_backward = static_cast<size_t>(search_window_bw);
    stream_params.search_window_forward  = static_cast<size_t>(search_window_fw);
    streamer_ = std::make_unique<TrajectoryStreamer>(stream_params);

    PathDeformer::Params deform_params;
    deform_params.max_deformation_step  = max_deformation_step;
    deform_params.max_total_deformation = max_total_deformation;
    deform_params.time_budget_ms        = time_budget_ms;
    deformer_ = std::make_unique<PathDeformer>(deform_params);

    robot_model_ = makeRobotModel(robot_type, kappa_max, articulation_limit);
    deformer_->setRobotModel(robot_model_.get());

    // --- Callback groups ---
    stream_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- QoS ---
    auto be_qos        = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    auto obs_qos       = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto cmd_qos       = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    auto rt_qos        = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    rclcpp::SubscriptionOptions stream_opts;
    stream_opts.callback_group = stream_group_;

    // --- Subscriptions ---
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, be_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); },
        stream_opts);

    obstacles_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic, obs_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onObstacles(msg); },
        stream_opts);

    traj_sub_ = create_subscription<norlab_controllers_msgs::msg::PathSequence>(
        trajectory_topic, transient_qos,
        [this](norlab_controllers_msgs::msg::PathSequence::SharedPtr msg) { onTrajectory(msg); });

    command_sub_ = create_subscription<std_msgs::msg::String>(
        command_topic, cmd_qos,
        [this](std_msgs::msg::String::SharedPtr msg) { onCommand(msg); });

    articulation_sub_ = create_subscription<std_msgs::msg::Float64>(
        articulation_topic, be_qos,
        [this](std_msgs::msg::Float64::SharedPtr msg) { onArticulationAngle(msg); });

    // --- Publishers ---
    control_plan_pub_  = create_publisher<nav_msgs::msg::Path>(control_local_plan_topic_, rt_qos);
    horizon_debug_pub_ = create_publisher<nav_msgs::msg::Path>(debug_horizon_topic,       rt_qos);
    deformed_debug_pub_= create_publisher<nav_msgs::msg::Path>(debug_deformed_topic, rt_qos);
    markers_pub_       = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic, rt_qos);
    replay_state_pub_  = create_publisher<wiln::msg::WilnState>(state_topic, transient_qos);
    diagnostics_pub_   = create_publisher<wiln::msg::ReplayDiagnostics>(diagnostics_topic, rt_qos);

    // --- Mapper clients ---
    enable_mapping_client_  = create_client<std_srvs::srv::Empty>(enable_mapping_svc);
    disable_mapping_client_ = create_client<std_srvs::srv::Empty>(disable_mapping_svc);

    // --- 10 Hz stream timer (always ticking, does nothing when IDLE) ---
    stream_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { streamLoop(); },
        stream_group_);

    publishState(wiln::msg::WilnState::IDLE, "ready");
    RCLCPP_INFO(get_logger(),
        "wiln_replay_node started. odom=%s trajectory=%s command=%s local_plan=%s model=%s, kappa_max=%.2f, speed=%.2f m/s",
        odom_topic.c_str(), trajectory_topic.c_str(), command_topic.c_str(), control_local_plan_topic_.c_str(),
        robot_model_->name().c_str(), kappa_max, trajectory_speed_);
}

// ---------------------------------------------------------------------------
// Subscriptions
// ---------------------------------------------------------------------------
void WilnReplayNode::onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto ptr = std::make_shared<const geometry_msgs::msg::Pose>(msg->pose.pose);
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ptr_ = std::move(ptr);
}

void WilnReplayNode::onObstacles(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto pts = fromPointCloud2(*msg);
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_ = std::move(pts);
}

void WilnReplayNode::onTrajectory(
    norlab_controllers_msgs::msg::PathSequence::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(traj_mutex_);
    cached_trajectory_ = *msg;
    traj_received_     = true;
}

void WilnReplayNode::onCommand(std_msgs::msg::String::SharedPtr msg)
{
    const std::string& cmd = msg->data;
    if (cmd == "play")        handlePlay();
    else if (cmd == "cancel") handleCancel();
}

void WilnReplayNode::onArticulationAngle(std_msgs::msg::Float64::SharedPtr msg)
{
    if (auto* mtt = dynamic_cast<MttModel*>(robot_model_.get()))
        mtt->updateArticulationAngle(msg->data);
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------
void WilnReplayNode::handlePlay()
{
    State expected = State::IDLE;
    if (!state_.compare_exchange_strong(expected, State::PREPARING)) {
        RCLCPP_WARN(get_logger(), "Cannot play — replay is already preparing or playing.");
        return;
    }

    // Get the cached trajectory
    norlab_controllers_msgs::msg::PathSequence traj;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (!traj_received_ || cached_trajectory_.paths.empty()) {
            RCLCPP_WARN(get_logger(), "No trajectory loaded — cannot play.");
            state_.store(State::IDLE);
            publishState(wiln::msg::WilnState::IDLE, "replay refused: no trajectory");
            return;
        }
        traj = cached_trajectory_;
    }

    // Determine play direction
    std::shared_ptr<const geometry_msgs::msg::Pose> pose_ptr;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_ptr = current_pose_ptr_;
    }
    if (!pose_ptr) {
        RCLCPP_WARN(get_logger(), "No pose available — cannot determine play direction.");
        state_.store(State::IDLE);
        publishState(wiln::msg::WilnState::IDLE, "replay refused: no pose");
        return;
    }

    double dist_to_start = distanceToPose(*pose_ptr, traj.paths.front().poses.front());
    double dist_to_end   = distanceToPose(*pose_ptr, traj.paths.back().poses.back());
    const double nearest_endpoint = std::min(dist_to_start, dist_to_end);
    if (nearest_endpoint > max_start_distance_m_) {
        RCLCPP_ERROR(get_logger(),
            "Replay refused: robot is %.2fm from the nearest route endpoint (limit %.2fm).",
            nearest_endpoint, max_start_distance_m_);
        state_.store(State::IDLE);
        publishState(wiln::msg::WilnState::IDLE,
            "replay refused: robot not at a route endpoint");
        return;
    }
    if (dist_to_end < dist_to_start) {
        RCLCPP_INFO(get_logger(), "Closer to end (%.2fm) — reversing trajectory.", dist_to_end);
        traj = reverseTrajectory(traj);
    } else {
        RCLCPP_INFO(get_logger(), "Playing forward from start (dist=%.2fm).", dist_to_start);
    }

    streamer_->setBaseTrajectory(traj);

    // Do not authorize replay motion until the mapper has acknowledged that map
    // insertion is disabled. Registration can take seconds, so fire the request
    // asynchronously and transition PREPARING -> PLAYING in its response.
    if (!disable_mapping_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(),
            "Replay refused: disable_mapping service unavailable; map freeze is not guaranteed.");
        state_.store(State::IDLE);
        publishState(wiln::msg::WilnState::IDLE, "replay refused: mapping service unavailable");
        return;
    }
    disable_mapping_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
            startAfterMappingDisabled();
        });
    RCLCPP_INFO(get_logger(), "Replay prepared; waiting for mapper freeze acknowledgement.");
}

void WilnReplayNode::startAfterMappingDisabled()
{
    State expected = State::PREPARING;
    if (!state_.compare_exchange_strong(expected, State::PLAYING)) {
        return;  // cancelled while the mapper request was pending
    }
    publishState(wiln::msg::WilnState::PLAYING, "replaying: mapping frozen");
    RCLCPP_INFO(get_logger(),
        "Mapper freeze acknowledged; replay started at %.2f m/s.", trajectory_speed_);
}

void WilnReplayNode::handleCancel()
{
    if (state_.load() == State::IDLE) return;
    stopReplay();
    RCLCPP_INFO(get_logger(), "Replay cancelled.");
}

void WilnReplayNode::stopReplay()
{
    state_.store(State::IDLE);

    // Clear fallback plan
    {
        std::lock_guard<std::mutex> lk(last_valid_plan_mutex_);
        last_valid_plan_.poses.clear();
    }

    // A teach/repeat session should normally keep the taught map immutable even
    // after completion/abort. The repeat supervisor explicitly re-enables map
    // insertion at the next teach_start.
    if (reenable_mapping_on_stop_
        && enable_mapping_client_->wait_for_service(std::chrono::milliseconds(0))) {
        enable_mapping_client_->async_send_request(
            std::make_shared<std_srvs::srv::Empty::Request>());
    }
    publishState(wiln::msg::WilnState::IDLE, "idle");
}

// ---------------------------------------------------------------------------
// 10 Hz stream loop
// ---------------------------------------------------------------------------
void WilnReplayNode::streamLoop()
{
    if (state_.load() != State::PLAYING) return;

    // Robot pose (pointer swap)
    std::shared_ptr<const geometry_msgs::msg::Pose> pose_ptr;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_ptr = current_pose_ptr_;
    }
    if (!pose_ptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No pose yet — skipping tick.");
        return;
    }

    // Local horizon
    TrajectoryStreamer::Diag stream_diag;
    auto horizon = streamer_->getLocalHorizon(*pose_ptr, &stream_diag);
    if (horizon.poses.empty()) {
        RCLCPP_INFO(get_logger(), "Trajectory completed (empty horizon).");
        stopReplay();
        return;
    }

    // Obstacles snapshot
    std::vector<Eigen::Vector3d> obstacles;
    {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        obstacles = latest_obstacles_;
    }

    nav_msgs::msg::Path safe_plan;
    PathDeformer::Diag  deform_diag;

    if (enable_deformation_) {
        // Deform (time-budgeted obstacle avoidance)
        safe_plan = deformer_->deform(horizon, obstacles, &deform_diag);
        safe_plan.header.stamp = now();

        if (deform_diag.used_fallback) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Deformer fallback (%.1f ms, budget_exceeded=%s)",
                deform_diag.deform_time_ms,
                deform_diag.time_budget_exceeded ? "yes" : "no");
        }

        // Fallback cache
        bool plan_valid = !deform_diag.used_fallback && safe_plan.poses.size() >= 3;
        {
            std::lock_guard<std::mutex> lk(last_valid_plan_mutex_);
            if (plan_valid) {
                last_valid_plan_ = safe_plan;
            } else if (!last_valid_plan_.poses.empty()) {
                safe_plan = last_valid_plan_;
            }
        }
    } else {
        // Deformer disabled — publish raw horizon as control plan (no obstacle deformation).
        // PathFollower is already configured to follow the global trajectory directly
        // (local_plan_topic points to a non-existent topic), so this publish is for
        // completeness / debug visibility only.
        safe_plan = horizon;
        safe_plan.header.stamp = now();
    }

    // Publish control plan
    control_plan_pub_->publish(safe_plan);

    // Debug topics
    horizon_debug_pub_->publish(horizon);
    deformed_debug_pub_->publish(safe_plan);

    // Diagnostics
    {
        wiln::msg::ReplayDiagnostics diag_msg;
        diag_msg.stamp               = now();
        diag_msg.dist_to_path_m      = stream_diag.dist_to_path_m;
        diag_msg.heading_error_rad   = stream_diag.heading_error_rad;
        diag_msg.closest_global_idx  = static_cast<uint32_t>(stream_diag.closest_global_idx);
        diag_msg.deform_time_ms      = deform_diag.deform_time_ms;
        diag_msg.obstacle_count      = static_cast<uint32_t>(obstacles.size());
        diag_msg.max_displacement_m  = deform_diag.max_displacement_m;
        diag_msg.deform_fallback     = deform_diag.used_fallback;
        diag_msg.time_budget_exceeded = deform_diag.time_budget_exceeded;
        diag_msg.kappa_max_current   = robot_model_ ? robot_model_->kappaMax() : 0.0;
        diagnostics_pub_->publish(diag_msg);
    }

    // Markers
    if (markers_pub_->get_subscription_count() > 0)
        publishMarkers(horizon, safe_plan, obstacles, stream_diag, deform_diag);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
norlab_controllers_msgs::msg::PathSequence WilnReplayNode::reverseTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& traj) const
{
    auto rev = traj;
    std::reverse(rev.paths.begin(), rev.paths.end());
    for (auto& path : rev.paths) {
        std::reverse(path.poses.begin(), path.poses.end());
        path.forward = !path.forward;
        for (auto& ps : path.poses) {
            tf2::Quaternion q;
            tf2::fromMsg(ps.pose.orientation, q);
            q = HALF_TURN_Z * q;
            q.normalize();
            ps.pose.orientation = tf2::toMsg(q);
        }
    }
    return rev;
}

double WilnReplayNode::distanceToPose(
    const geometry_msgs::msg::Pose&       from,
    const geometry_msgs::msg::PoseStamped& to) const
{
    double dx = from.position.x - to.pose.position.x;
    double dy = from.position.y - to.pose.position.y;
    double dz = from.position.z - to.pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<Eigen::Vector3d> WilnReplayNode::fromPointCloud2(
    const sensor_msgs::msg::PointCloud2& msg)
{
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg.width * msg.height);

    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
        pts.emplace_back(*ix, *iy, *iz);
    }
    return pts;
}

void WilnReplayNode::publishState(uint8_t state_code, const std::string& detail)
{
    wiln::msg::WilnState msg;
    msg.stamp  = now();
    msg.state  = state_code;
    msg.detail = detail;
    replay_state_pub_->publish(msg);
}

void WilnReplayNode::publishMarkers(
    const nav_msgs::msg::Path&          horizon,
    const nav_msgs::msg::Path&          deformed,
    const std::vector<Eigen::Vector3d>& obstacles,
    const TrajectoryStreamer::Diag&     stream_diag,
    const PathDeformer::Diag&           deform_diag)
{
    visualization_msgs::msg::MarkerArray array;
    auto t = now();
    const std::string frame = deformed.header.frame_id.empty() ? "map" : deformed.header.frame_id;

    auto make_strip = [&](int id, float r, float g, float b,
                          const nav_msgs::msg::Path& path) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp    = t;
        m.ns              = "wiln";
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.scale.x         = 0.08f;
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

    array.markers.push_back(make_strip(0, 0.0f, 1.0f, 1.0f, horizon));   // cyan: raw horizon
    array.markers.push_back(make_strip(1, 1.0f, 1.0f, 0.0f, deformed));  // yellow: deformed

    // Obstacles (red cubes near deformed path)
    {
        visualization_msgs::msg::Marker obs_m;
        obs_m.header.frame_id = frame;
        obs_m.header.stamp    = t;
        obs_m.ns              = "wiln";
        obs_m.id              = 2;
        obs_m.type            = visualization_msgs::msg::Marker::CUBE_LIST;
        obs_m.action          = visualization_msgs::msg::Marker::ADD;
        obs_m.scale.x = obs_m.scale.y = obs_m.scale.z = 0.12f;
        obs_m.lifetime = rclcpp::Duration::from_seconds(0.5);
        constexpr double rep_dist = 1.5;
        for (const auto& o : obstacles) {
            bool near = false;
            for (size_t k = 0; k < deformed.poses.size(); k += 5) {
                const auto& pp = deformed.poses[k].pose.position;
                double d2 = (o.x()-pp.x)*(o.x()-pp.x) + (o.y()-pp.y)*(o.y()-pp.y);
                if (d2 < (rep_dist*3.0)*(rep_dist*3.0)) { near = true; break; }
            }
            if (!near) continue;
            geometry_msgs::msg::Point pt;
            pt.x = o.x(); pt.y = o.y(); pt.z = o.z();
            obs_m.points.push_back(pt);
            std_msgs::msg::ColorRGBA c;
            c.r = 1.0f; c.g = 0.3f; c.b = 0.1f; c.a = 0.75f;
            obs_m.colors.push_back(c);
        }
        array.markers.push_back(obs_m);
    }

    // Status text
    {
        visualization_msgs::msg::Marker txt;
        txt.header.frame_id = frame;
        txt.header.stamp    = t;
        txt.ns              = "wiln";
        txt.id              = 4;
        txt.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action          = visualization_msgs::msg::Marker::ADD;
        txt.scale.z         = 0.35f;
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;
        txt.lifetime = rclcpp::Duration::from_seconds(1.0);
        if (!deformed.poses.empty()) {
            auto& p = deformed.poses[deformed.poses.size()/2].pose.position;
            txt.pose.position.x = p.x;
            txt.pose.position.y = p.y;
            txt.pose.position.z = p.z + 0.7;
        }
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "WILN:%s  model:%s\n"
            "dist:%.2fm  hdg:%.1f°  idx:%zu\n"
            "deform:%.1fms  obs:%zu  disp:%.2fm%s",
            (state_.load() == State::PLAYING ? "PLAYING" : "IDLE"),
            robot_model_ ? robot_model_->name().c_str() : "?",
            stream_diag.dist_to_path_m,
            stream_diag.heading_error_rad * 180.0 / M_PI,
            stream_diag.closest_global_idx,
            deform_diag.deform_time_ms,
            obstacles.size(),
            deform_diag.max_displacement_m,
            deform_diag.used_fallback ? " [FALLBACK]" : "");
        txt.text = buf;
        array.markers.push_back(txt);
    }

    markers_pub_->publish(array);
}

} // namespace wiln

// ---------------------------------------------------------------------------
// main -- MultiThreadedExecutor (stream loop on dedicated thread)
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wiln::WilnReplayNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
