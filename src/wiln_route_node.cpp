#include "wiln/WilnRouteNode.hpp"

#include <cmath>
#include <limits>

namespace wiln {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WilnRouteNode::WilnRouteNode() : Node("wiln_route_node")
{
    // --- Parameters ---
    save_map_service_ = declare_parameter("save_map_service", std::string("/mapping/save_map"));
    load_map_service_ = declare_parameter("load_map_service", std::string("/mapping/load_map"));
    create_parent_dirs_ = declare_parameter("create_parent_dirs", true);
    max_route_step_m_ = declare_parameter("max_route_step_m", 1.0);
    max_route_yaw_step_rad_ = declare_parameter("max_route_yaw_step_rad", 0.8);
    max_route_z_span_m_ = declare_parameter("max_route_z_span_m", 2.0);
    const std::string command_topic = declare_parameter("command_topic", std::string("/wiln/command"));
    const std::string trajectory_topic = declare_parameter("trajectory_topic", std::string("/wiln/trajectory"));
    const std::string global_plan_topic = declare_parameter("global_plan_topic", std::string("/wiln/global_plan"));
    const std::string state_topic = declare_parameter("state_topic", std::string("/wiln/route/state"));

    // --- QoS ---
    auto cmd_qos       = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    // --- Subscriptions ---
    command_sub_ = create_subscription<std_msgs::msg::String>(
        command_topic, cmd_qos,
        [this](std_msgs::msg::String::SharedPtr msg) { onCommand(msg); });

    traj_sub_ = create_subscription<norlab_controllers_msgs::msg::PathSequence>(
        trajectory_topic, transient_qos,
        [this](norlab_controllers_msgs::msg::PathSequence::SharedPtr msg) { onTrajectory(msg); });

    // --- Publishers ---
    trajectory_pub_  = create_publisher<norlab_controllers_msgs::msg::PathSequence>(
        trajectory_topic, transient_qos);
    global_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
        global_plan_topic, transient_qos);
    state_pub_       = create_publisher<wiln::msg::WilnState>(
        state_topic, transient_qos);

    // --- Mapper clients (async, non-blocking) ---
    save_map_client_ = create_client<SaveMap>(save_map_service_);
    load_map_client_ = create_client<LoadMap>(load_map_service_);

    publishState(wiln::msg::WilnState::IDLE, "ready");
    RCLCPP_INFO(get_logger(), "wiln_route_node started. command=%s trajectory=%s save_map=%s load_map=%s",
        command_topic.c_str(), trajectory_topic.c_str(), save_map_service_.c_str(), load_map_service_.c_str());
}

// ---------------------------------------------------------------------------
// Cache latest trajectory (needed for save)
// ---------------------------------------------------------------------------
void WilnRouteNode::onTrajectory(
    norlab_controllers_msgs::msg::PathSequence::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(traj_mutex_);
    cached_trajectory_ = *msg;
    traj_received_     = true;
}

// ---------------------------------------------------------------------------
// Command dispatcher
// ---------------------------------------------------------------------------
void WilnRouteNode::onCommand(std_msgs::msg::String::SharedPtr msg)
{
    const std::string& cmd = msg->data;
    if (cmd.rfind("save:", 0) == 0) {
        handleSave(cmd.substr(5));
    } else if (cmd.rfind("load:", 0) == 0) {
        handleLoad(cmd.substr(5));
    }
}

// ---------------------------------------------------------------------------
// Save handler
// ---------------------------------------------------------------------------
void WilnRouteNode::handleSave(const std::string& filepath)
{
    if (filepath.empty()) {
        RCLCPP_ERROR(get_logger(), "save: empty filepath.");
        publishState(wiln::msg::WilnState::IDLE, "save failed: empty filepath");
        return;
    }

    norlab_controllers_msgs::msg::PathSequence traj;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        size_t pose_count = 0;
        std::string reason;
        if (!traj_received_ || !trajectoryUsable(cached_trajectory_, &pose_count, &reason)) {
            RCLCPP_WARN(get_logger(), "save: no usable trajectory available: %s", reason.c_str());
            publishState(wiln::msg::WilnState::IDLE, "save failed: " + reason);
            return;
        }
        traj = cached_trajectory_;
    }

    std::filesystem::path out_path(filepath);
    if (create_parent_dirs_ && !out_path.parent_path().empty()) {
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to create route directory %s: %s",
                out_path.parent_path().string().c_str(), ec.message().c_str());
            publishState(wiln::msg::WilnState::IDLE, "save failed: mkdir");
            return;
        }
    }

    publishState(wiln::msg::WilnState::SAVING, "saving " + filepath);
    RCLCPP_INFO(get_logger(), "Saving trajectory to %s ...", filepath.c_str());

    const bool ok = TrajectoryManager::saveLTR(filepath, traj);
    if (ok) {
        RCLCPP_INFO(get_logger(), "Trajectory saved to %s", filepath.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to save trajectory to %s", filepath.c_str());
        publishState(wiln::msg::WilnState::IDLE, "save failed: trajectory write");
        return;
    }

    // A saved route is a trajectory + the exact teach map. Do not announce
    // success until the mapper service has returned and the VTK file exists;
    // otherwise an immediate replay can load a stale or partially-written map.
    const std::string vtk_path = filepath + ".vtk";
    const std::string route_name = std::filesystem::path(filepath).filename().string();
    if (save_map_client_->wait_for_service(std::chrono::milliseconds(0))) {
        auto req = std::make_shared<SaveMap::Request>();
        req->map_file_name.data = vtk_path;
        save_map_client_->async_send_request(
            req, [this, vtk_path, route_name](rclcpp::Client<SaveMap>::SharedFuture future) {
                try {
                    (void)future.get();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Map save failed for %s: %s",
                        vtk_path.c_str(), e.what());
                    publishState(wiln::msg::WilnState::IDLE,
                        "save failed: mapper service error", route_name);
                    return;
                }
                std::error_code ec;
                const bool map_ready = std::filesystem::exists(vtk_path, ec) &&
                    !ec && std::filesystem::file_size(vtk_path, ec) > 0 && !ec;
                if (!map_ready) {
                    RCLCPP_ERROR(get_logger(),
                        "Mapper returned but route map is missing or empty: %s",
                        vtk_path.c_str());
                    publishState(wiln::msg::WilnState::IDLE,
                        "save failed: map missing", route_name);
                    return;
                }
                RCLCPP_INFO(get_logger(), "Route map saved completely: %s", vtk_path.c_str());
                publishState(wiln::msg::WilnState::IDLE, "saved", route_name);
            });
    } else {
        RCLCPP_ERROR(get_logger(),
            "Mapper save service unavailable — route is incomplete and was not armed as saved.");
        publishState(wiln::msg::WilnState::IDLE,
            "save failed: mapper unavailable", route_name);
    }
}

// ---------------------------------------------------------------------------
// Load handler
// ---------------------------------------------------------------------------
void WilnRouteNode::handleLoad(const std::string& filepath)
{
    if (filepath.empty()) {
        RCLCPP_ERROR(get_logger(), "load: empty filepath.");
        publishState(wiln::msg::WilnState::IDLE, "load failed: empty filepath");
        return;
    }

    publishState(wiln::msg::WilnState::LOADING, "loading " + filepath);
    RCLCPP_INFO(get_logger(), "Loading trajectory from %s ...", filepath.c_str());

    norlab_controllers_msgs::msg::PathSequence traj;
    size_t pose_count = 0;
    std::string reason;
    if (!TrajectoryManager::loadLTR(filepath, traj) || !trajectoryUsable(traj, &pose_count, &reason)) {
        RCLCPP_ERROR(get_logger(), "Failed to load LTR from %s: %s", filepath.c_str(), reason.c_str());
        publishState(wiln::msg::WilnState::IDLE, "load failed: " + reason);
        return;
    }

    // The map establishes the coordinate frame used by the saved trajectory.
    // Do not publish the trajectory (and therefore do not let replay arm) until
    // the mapper service has returned. Previously the trajectory was published
    // first, so replay selected its direction from the old map pose while the
    // mapper was still replacing the map underneath it.
    const std::string vtk_path = filepath + ".vtk";
    if (load_map_client_->wait_for_service(std::chrono::milliseconds(0))
        && std::filesystem::exists(vtk_path))
    {
        auto req = std::make_shared<LoadMap::Request>();
        req->map_file_name.data = vtk_path;
        if (!traj.paths.empty() && !traj.paths.front().poses.empty())
            req->pose = traj.paths.front().poses.front().pose;
        RCLCPP_INFO(get_logger(), "Map load started: %s", vtk_path.c_str());
        load_map_client_->async_send_request(
            req,
            [this, traj, pose_count, filepath, vtk_path](
                rclcpp::Client<LoadMap>::SharedFuture future) {
                try {
                    (void)future.get();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Map load failed for %s: %s",
                        vtk_path.c_str(), e.what());
                    publishState(wiln::msg::WilnState::IDLE,
                        "load failed: mapper service error");
                    return;
                }
                RCLCPP_INFO(get_logger(), "Map load completed: %s", vtk_path.c_str());
                finishLoadedTrajectory(traj, pose_count, filepath);
            });
        return;
    } else if (!std::filesystem::exists(vtk_path)) {
        RCLCPP_WARN(get_logger(), "No VTK map at %s — loaded trajectory only.", vtk_path.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "Mapper load service unavailable — loaded trajectory only.");
    }

    finishLoadedTrajectory(traj, pose_count, filepath);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void WilnRouteNode::publishTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& traj)
{
    trajectory_pub_->publish(traj);

    nav_msgs::msg::Path global;
    global.header = traj.header;
    global.header.stamp = now();
    for (const auto& path : traj.paths)
        for (const auto& ps : path.poses)
            global.poses.push_back(ps);
    global_plan_pub_->publish(global);
}

void WilnRouteNode::finishLoadedTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& traj,
    size_t pose_count,
    const std::string& filepath)
{
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        cached_trajectory_ = traj;
        traj_received_     = true;
    }
    publishTrajectory(traj);
    RCLCPP_INFO(get_logger(), "Trajectory loaded after map: %zu segment(s), %zu poses.",
        traj.paths.size(), pose_count);
    const std::string route_name = std::filesystem::path(filepath).filename().string();
    publishState(wiln::msg::WilnState::IDLE, "loaded", route_name);
}

void WilnRouteNode::publishState(uint8_t state_code, const std::string& detail,
                                  const std::string& route_name)
{
    wiln::msg::WilnState msg;
    msg.stamp        = now();
    msg.state        = state_code;
    msg.detail       = detail;
    msg.active_route = route_name;
    state_pub_->publish(msg);
}

bool WilnRouteNode::trajectoryUsable(
    const norlab_controllers_msgs::msg::PathSequence& traj,
    size_t* pose_count,
    std::string* reason) const
{
    size_t total = 0;
    size_t non_empty_segments = 0;
    double z_min = std::numeric_limits<double>::infinity();
    double z_max = -std::numeric_limits<double>::infinity();
    const geometry_msgs::msg::PoseStamped* previous_route_pose = nullptr;

    for (const auto& path : traj.paths) {
        if (!path.poses.empty()) {
            non_empty_segments++;
            total += path.poses.size();
        }
        for (const auto& pose : path.poses) {
            z_min = std::min(z_min, pose.pose.position.z);
            z_max = std::max(z_max, pose.pose.position.z);
            if (previous_route_pose == nullptr) {
                previous_route_pose = &pose;
                continue;
            }
            // Validate across segment boundaries as well as within a segment.
            // ICP gap recovery represents a discontinuity as a new segment; if
            // boundaries are skipped here, a multi-metre teleport can be saved
            // as a seemingly valid teach route.
            const auto& previous = *previous_route_pose;
            const double dx = pose.pose.position.x - previous.pose.position.x;
            const double dy = pose.pose.position.y - previous.pose.position.y;
            const double dz = pose.pose.position.z - previous.pose.position.z;
            const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
            const double yaw_step = std::abs(wrapToPi(
                yawFromQuaternion(pose.pose.orientation) -
                yawFromQuaternion(previous.pose.orientation)));
            if (step > max_route_step_m_) {
                if (reason) {
                    *reason = "large route step " + std::to_string(step) + " m";
                }
                return false;
            }
            if (yaw_step > max_route_yaw_step_rad_) {
                if (reason) {
                    *reason = "large route yaw step " + std::to_string(yaw_step) + " rad";
                }
                return false;
            }
            previous_route_pose = &pose;
        }
    }
    if (pose_count) {
        *pose_count = total;
    }
    if (non_empty_segments == 0 || total < 2) {
        if (reason) {
            *reason = "too few poses";
        }
        return false;
    }
    if (std::isfinite(z_min) && std::isfinite(z_max) &&
        (z_max - z_min) > max_route_z_span_m_) {
        if (reason) {
            *reason = "large route z span " + std::to_string(z_max - z_min) + " m";
        }
        return false;
    }
    if (reason) {
        *reason = "ok";
    }
    return true;
}

double WilnRouteNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

double WilnRouteNode::wrapToPi(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace wiln

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wiln::WilnRouteNode>());
    rclcpp::shutdown();
    return 0;
}
