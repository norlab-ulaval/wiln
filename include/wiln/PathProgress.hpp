#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace wiln {

/** Monotone progress and lookahead selection on a sampled path. */
struct PathProgress
{
    int nearest_index{0};
    int target_index{0};
    double nearest_distance_m{std::numeric_limits<double>::infinity()};
};

inline double pathPointDistance(
    const geometry_msgs::msg::Pose& robot,
    const geometry_msgs::msg::Pose& point)
{
    return std::hypot(
        robot.position.x - point.position.x,
        robot.position.y - point.position.y);
}

/**
 * Recover progress after a missed waypoint, then select a target by arc length.
 *
 * The old controller only advanced when it passed within a small radius of the
 * current waypoint.  Missing one sample permanently left that sample behind the
 * robot, so the controller eventually saturated its articulation trying to turn
 * back.  This selector searches forward from the last confirmed progress index,
 * never moves progress backward, and keeps the steering target ahead by a
 * physical lookahead distance rather than by a number of samples.
 *
 * A bounded search window avoids jumping to a distant branch where a route
 * crosses itself.  Initial route acquisition should provide the first progress
 * index using a global nearest-point search.
 */
inline PathProgress selectPathProgress(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    int previous_progress_index,
    const geometry_msgs::msg::Pose& robot_pose,
    double lookahead_distance_m,
    int search_ahead_points)
{
    PathProgress result;
    if (poses.empty()) {
        return result;
    }

    const int last = static_cast<int>(poses.size()) - 1;
    const int begin = std::clamp(previous_progress_index, 0, last);
    const int window = std::max(search_ahead_points, 1);
    const int end = std::min(last, begin + window);

    result.nearest_index = begin;
    result.nearest_distance_m = pathPointDistance(robot_pose, poses[begin].pose);
    for (int i = begin + 1; i <= end; ++i) {
        const double distance = pathPointDistance(robot_pose, poses[i].pose);
        // Strictly better only: ties keep the earliest branch and make progress
        // deterministic at crossings and on duplicated samples.
        if (distance + 1e-9 < result.nearest_distance_m) {
            result.nearest_distance_m = distance;
            result.nearest_index = i;
        }
    }

    result.target_index = result.nearest_index;
    double arc_length = 0.0;
    const double requested_lookahead = std::max(lookahead_distance_m, 0.0);
    while (result.target_index < last && arc_length < requested_lookahead) {
        const auto& from = poses[result.target_index].pose.position;
        const auto& to = poses[result.target_index + 1].pose.position;
        arc_length += std::hypot(to.x - from.x, to.y - from.y);
        ++result.target_index;
    }
    return result;
}

}  // namespace wiln
