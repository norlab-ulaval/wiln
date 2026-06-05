#include "wiln/TeachRecorder.hpp"
#include "wiln/SE3Utils.hpp"
#include <cmath>
#include <tf2/utils.h>

namespace wiln {

TeachRecorder::TeachRecorder(const Params& params) : params_(params) {}

void TeachRecorder::start() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    trajectory_.paths.clear();
    rejected_jumps_ = 0;
    gap_recovery_count_ = 0;
    recording_ = true;
}

void TeachRecorder::stop() {
    recording_ = false;
}

void TeachRecorder::clear() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    trajectory_.paths.clear();
}

void TeachRecorder::addPose(const geometry_msgs::msg::PoseStamped& pose, bool forward) {
    if (!recording_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (trajectory_.paths.empty()) {
        norlab_controllers_msgs::msg::DirectionalPath path;
        path.forward = forward;
        path.header = pose.header;
        path.poses.push_back(pose);
        trajectory_.paths.push_back(path);
        trajectory_.header = pose.header;
        last_forward_direction_ = forward;
        return;
    }

    auto& current_path = trajectory_.paths.back();
    const auto& last_pose = current_path.poses.back();

    double dist = computeDistance(pose.pose.position, last_pose.pose.position);
    double angle_diff = std::abs(extractYaw(pose.pose.orientation) - extractYaw(last_pose.pose.orientation));
    if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

    if (dist > params_.max_record_jump_m || angle_diff > params_.max_record_yaw_jump_rad) {
        ++rejected_jumps_;

        // 11A: Gap recovery — instead of truncating the trajectory permanently, detect
        // when ICP has stabilised at a new consistent location after a dropout.
        // Track consecutive poses that stay near a stable anchor point (= ICP just resumed).
        // After 5 such poses, start a new trajectory segment and reconnect.
        // The gap appears as a direction-change marker between segments, which replay
        // handles as a "change direction" event (safe — robot is stationary between segments).
        const double stable_radius = params_.min_dist_between_poses * 3.0;
        if (gap_recovery_count_ == 0) {
            // First rejected pose — set anchor at this position.
            gap_recovery_anchor_ = pose;
            gap_recovery_count_ = 1;
        } else {
            const double dist_from_anchor = computeDistance(
                pose.pose.position, gap_recovery_anchor_.pose.position);
            if (dist_from_anchor <= stable_radius) {
                // Pose is near anchor — ICP is publishing stable estimates at this location.
                ++gap_recovery_count_;
            } else {
                // Pose jumped again — reset anchor to current position and restart count.
                gap_recovery_anchor_ = pose;
                gap_recovery_count_ = 1;
            }
        }

        // After 5 stable poses: ICP has recovered. Reconnect by starting a new segment.
        static constexpr uint32_t kRecoveryStablePoses = 5;
        if (gap_recovery_count_ >= kRecoveryStablePoses) {
            // Insert a new DirectionalPath starting at the recovery anchor.
            // The gap (robot teleport) is represented as a segment boundary.
            norlab_controllers_msgs::msg::DirectionalPath recovery_path;
            recovery_path.forward = forward;
            recovery_path.header  = gap_recovery_anchor_.header;
            recovery_path.poses.push_back(gap_recovery_anchor_);
            {
                // Already under data_mutex_ (called from addPose lock scope).
                trajectory_.paths.push_back(recovery_path);
            }
            last_forward_direction_ = forward;
            gap_recovery_count_ = 0;
        }
        return;
    }

    // Normal pose — reset gap recovery state.
    gap_recovery_count_ = 0;

    // Change direction?
    if (forward != last_forward_direction_) {
        norlab_controllers_msgs::msg::DirectionalPath new_path;
        new_path.forward = forward;
        new_path.header = pose.header;
        new_path.poses.push_back(pose);
        trajectory_.paths.push_back(new_path);
        last_forward_direction_ = forward;
    }
    // Large rotation?
    else if (angle_diff > params_.min_angle_between_poses) {
        norlab_controllers_msgs::msg::DirectionalPath new_path;
        new_path.forward = forward;
        new_path.header = pose.header;
        geometry_msgs::msg::PoseStamped rotated_pose = pose;
        rotated_pose.pose.position = last_pose.pose.position;
        new_path.poses.push_back(rotated_pose);
        trajectory_.paths.push_back(new_path);
    }
    // Just distance
    else if (dist >= params_.min_dist_between_poses) {
        current_path.poses.push_back(pose);
    }
}

norlab_controllers_msgs::msg::PathSequence TeachRecorder::getTrajectory() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return trajectory_;
}

void TeachRecorder::setTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    trajectory_ = traj;
    if (!traj.paths.empty()) {
        last_forward_direction_ = traj.paths.back().forward;
    }
}

bool TeachRecorder::smooth() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const int W = params_.smoothing_window;
    if (W <= 0) {
        return true;
    }

    const auto original_trajectory = trajectory_;

    for (auto& path : trajectory_.paths) {
        if (static_cast<int>(path.poses.size()) < 2 * W + 1) continue;

        std::vector<geometry_msgs::msg::PoseStamped> smoothed_poses = path.poses;
        const double inv_window = 1.0 / static_cast<double>(2 * W + 1);

        for (size_t i = static_cast<size_t>(W);
             i < path.poses.size() - static_cast<size_t>(W); ++i) {

            // Fréchet mean on SE(3): iterative left-perturbation averaging.
            // 1. Initialise T_mean at the central pose.
            Eigen::Matrix4d T_mean = se3::fromPose(path.poses[i].pose);

            // 2. Iterate until convergence (3 iterations is sufficient for a
            //    local window of Gaussian-distributed poses).
            for (int iter = 0; iter < 3; ++iter) {
                Eigen::Matrix4d T_mean_inv = se3::InvSE3(T_mean);

                // Accumulate sum of Log(T_mean⁻¹ · T_j) in se(3)
                Eigen::Vector<double,6> xi_sum = Eigen::Vector<double,6>::Zero();
                for (int k = -W; k <= W; ++k) {
                    Eigen::Matrix4d T_j = se3::fromPose(path.poses[i + k].pose);
                    xi_sum += se3::LogSE3(T_mean_inv * T_j);
                }
                Eigen::Vector<double,6> xi_mean = xi_sum * inv_window;

                // Update T_mean = T_mean · Exp(xi_mean)
                T_mean = T_mean * se3::ExpSE3(xi_mean);

                // Early exit if correction is negligible
                if (xi_mean.norm() < 1e-8) break;
            }

            smoothed_poses[i].pose = se3::toPose<geometry_msgs::msg::Pose>(T_mean);
        }
        path.poses = smoothed_poses;
    }

    if (trajectoryHasLargeJump(trajectory_)) {
        trajectory_ = original_trajectory;
        return false;
    }

    return true;
}

double TeachRecorder::computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double TeachRecorder::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    return tf2::getYaw(tf_q);
}

bool TeachRecorder::trajectoryHasLargeJump(
    const norlab_controllers_msgs::msg::PathSequence& trajectory) const
{
    for (const auto& path : trajectory.paths) {
        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto& previous = path.poses[i - 1];
            const auto& current = path.poses[i];
            const double dist = computeDistance(current.pose.position, previous.pose.position);
            double angle_diff = std::abs(
                extractYaw(current.pose.orientation) - extractYaw(previous.pose.orientation));
            if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
            if (dist > params_.max_record_jump_m || angle_diff > params_.max_record_yaw_jump_rad) {
                return true;
            }
        }
    }
    return false;
}

} // namespace wiln
