#include "wiln/TrajectoryStreamer.hpp"

#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace wiln {

TrajectoryStreamer::TrajectoryStreamer(const Params& params) : params_(params) {}

void TrajectoryStreamer::setBaseTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    base_trajectory_ = traj;
}

nav_msgs::msg::Path TrajectoryStreamer::getLocalHorizon(const geometry_msgs::msg::Pose& current_pose) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    nav_msgs::msg::Path horizon;
    horizon.header.frame_id = base_trajectory_.header.frame_id;
    horizon.header.stamp = rclcpp::Clock().now();

    if (base_trajectory_.paths.empty()) return horizon;

    size_t closest_path_idx = 0;
    size_t closest_pose_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < base_trajectory_.paths.size(); ++i) {
        for (size_t j = 0; j < base_trajectory_.paths[i].poses.size(); ++j) {
            const auto& p = base_trajectory_.paths[i].poses[j].pose.position;
            double d = std::sqrt(std::pow(p.x - current_pose.position.x, 2) +
                                 std::pow(p.y - current_pose.position.y, 2));
            if (d < min_dist) {
                min_dist = d;
                closest_path_idx = i;
                closest_pose_idx = j;
            }
        }
    }

    // Extract next points until horizon_length is reached
    double accumulated_dist = 0;
    size_t p_idx = closest_path_idx;
    size_t s_idx = closest_pose_idx;

    while (accumulated_dist < params_.horizon_length && p_idx < base_trajectory_.paths.size()) {
        const auto& pose = base_trajectory_.paths[p_idx].poses[s_idx];
        horizon.poses.push_back(pose);

        // Move to next point
        size_t next_s = s_idx + 1;
        size_t next_p = p_idx;
        if (next_s >= base_trajectory_.paths[p_idx].poses.size()) {
            next_s = 0;
            next_p++;
        }

        if (next_p < base_trajectory_.paths.size()) {
            const auto& p1 = base_trajectory_.paths[p_idx].poses[s_idx].pose.position;
            const auto& p2 = base_trajectory_.paths[next_p].poses[next_s].pose.position;
            accumulated_dist += std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        }
        
        p_idx = next_p;
        s_idx = next_s;
    }

    return horizon;
}

} // namespace wiln
