#include "wiln/TeachRecorder.hpp"
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

void TeachRecorder::smooth() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& path : trajectory_.paths) {
        if (path.poses.size() < 2 * params_.smoothing_window + 1) continue;

        std::vector<geometry_msgs::msg::PoseStamped> smoothed_poses = path.poses;
        for (size_t i = params_.smoothing_window; i < path.poses.size() - params_.smoothing_window; ++i) {
            double sum_x = 0, sum_y = 0, sum_z = 0;
            for (int k = -params_.smoothing_window; k <= params_.smoothing_window; ++k) {
                sum_x += path.poses[i + k].pose.position.x;
                sum_y += path.poses[i + k].pose.position.y;
                sum_z += path.poses[i + k].pose.position.z;
            }
            smoothed_poses[i].pose.position.x = sum_x / (2 * params_.smoothing_window + 1);
            smoothed_poses[i].pose.position.y = sum_y / (2 * params_.smoothing_window + 1);
            smoothed_poses[i].pose.position.z = sum_z / (2 * params_.smoothing_window + 1);
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
