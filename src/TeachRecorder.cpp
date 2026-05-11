#include "wiln/TeachRecorder.hpp"
#include "wiln/SE3Utils.hpp"
#include <cmath>
#include <tf2/utils.h>

namespace wiln {

TeachRecorder::TeachRecorder(const Params& params) : params_(params) {}

void TeachRecorder::start() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    trajectory_.paths.clear();
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

void TeachRecorder::smooth() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const int W = params_.smoothing_window;

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
}

double TeachRecorder::computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double TeachRecorder::extractYaw(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    return tf2::getYaw(tf_q);
}

} // namespace wiln
