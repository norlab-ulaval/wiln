#include "wiln/TrajectoryStreamer.hpp"

#include <cmath>
#include <limits>

namespace wiln {

TrajectoryStreamer::TrajectoryStreamer(const Params& params) : params_(params) {}

// ---------------------------------------------------------------------------
void TrajectoryStreamer::setBaseTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& traj)
{
    std::lock_guard<std::mutex> lock(traj_mutex_);
    base_trajectory_  = traj;
    frame_id_         = traj.header.frame_id;
    last_closest_idx_ = 0;
    buildFlatIndex(traj);
}

// ---------------------------------------------------------------------------
void TrajectoryStreamer::buildFlatIndex(
    const norlab_controllers_msgs::msg::PathSequence& traj)
{
    flat_index_.clear();
    for (size_t seg = 0; seg < traj.paths.size(); ++seg) {
        for (const auto& ps : traj.paths[seg].poses) {
            flat_index_.push_back({ps, seg});
        }
    }
}

// ---------------------------------------------------------------------------
nav_msgs::msg::Path TrajectoryStreamer::getLocalHorizon(
    const geometry_msgs::msg::Pose& current_pose, Diag* diag)
{
    std::lock_guard<std::mutex> lock(traj_mutex_);

    nav_msgs::msg::Path horizon;
    horizon.header.frame_id = frame_id_;

    if (flat_index_.empty()) return horizon;

    // --- Find closest pose (monotonic window search) ---
    bool full_search = false;
    size_t closest = findClosestInWindow(current_pose, &full_search);
    last_closest_idx_ = closest;

    // --- Diagnostics ---
    if (diag) {
        const auto& cp = flat_index_[closest].pose.pose.position;
        double dx = current_pose.position.x - cp.x;
        double dy = current_pose.position.y - cp.y;
        diag->closest_global_idx = closest;
        diag->dist_to_path_m     = std::sqrt(dx*dx + dy*dy);
        diag->full_search_used   = full_search;

        // Heading error: angle between robot heading and path tangent
        if (closest + 1 < flat_index_.size()) {
            const auto& np = flat_index_[closest + 1].pose.pose.position;
            double tangent_yaw = std::atan2(np.y - cp.y, np.x - cp.x);
            const auto& q = current_pose.orientation;
            double robot_yaw = std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            double heading_err = robot_yaw - tangent_yaw;
            while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
            while (heading_err < -M_PI) heading_err += 2.0 * M_PI;
            diag->heading_error_rad = heading_err;
        }
    }

    // --- Extract local horizon forward from closest ---
    double accumulated = 0.0;
    size_t idx = closest;
    const size_t active_segment = flat_index_[closest].path_segment;

    while (idx < flat_index_.size()) {
        // A DirectionalPath boundary is a real stop/direction transition, not
        // a geometric edge to concatenate into the local horizon. Crossing it
        // created long spikes between independent Teach segments.
        if (flat_index_[idx].path_segment != active_segment) break;
        horizon.poses.push_back(flat_index_[idx].pose);

        if (idx + 1 < flat_index_.size() &&
            flat_index_[idx + 1].path_segment == active_segment) {
            const auto& pa = flat_index_[idx].pose.pose.position;
            const auto& pb = flat_index_[idx + 1].pose.pose.position;
            double dx = pb.x - pa.x;
            double dy = pb.y - pa.y;
            accumulated += std::sqrt(dx*dx + dy*dy);
        } else {
            break;
        }

        if (accumulated >= params_.horizon_length) break;
        ++idx;
    }

    if (diag) diag->horizon_points = horizon.poses.size();
    return horizon;
}

// ---------------------------------------------------------------------------
size_t TrajectoryStreamer::findClosestInWindow(
    const geometry_msgs::msg::Pose& pose, bool* full_search_used) const
{
    if (full_search_used) *full_search_used = false;
    if (flat_index_.empty()) return 0;

    const size_t N = flat_index_.size();

    // Window search boundaries
    size_t win_start = (last_closest_idx_ > params_.search_window_backward)
                     ? last_closest_idx_ - params_.search_window_backward : 0;
    size_t win_end   = std::min(last_closest_idx_ + params_.search_window_forward, N - 1);

    double  min_dist = std::numeric_limits<double>::max();
    size_t  best_idx = last_closest_idx_;

    for (size_t i = win_start; i <= win_end; ++i) {
        const auto& p = flat_index_[i].pose.pose.position;
        double dx = p.x - pose.position.x;
        double dy = p.y - pose.position.y;
        double d  = dx*dx + dy*dy;
        if (d < min_dist) { min_dist = d; best_idx = i; }
    }

    // Fallback: if the robot appears to be far from the window (e.g., after
    // a large localization jump), perform a full search and reset the tracker.
    double best_dist = std::sqrt(min_dist);
    if (best_dist > 3.0) {  // [m] threshold for full-search reset
        if (full_search_used) *full_search_used = true;
        min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < N; ++i) {
            const auto& p = flat_index_[i].pose.pose.position;
            double dx = p.x - pose.position.x;
            double dy = p.y - pose.position.y;
            double d  = dx*dx + dy*dy;
            if (d < min_dist) { min_dist = d; best_idx = i; }
        }
    }

    return best_idx;
}

} // namespace wiln
