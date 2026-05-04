#pragma once

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <mutex>

namespace wiln {

/**
 * @brief Manages the "Lookahead Horizon" for the repeat phase.
 * Extract a local chunk of the path for streaming.
 */
class TrajectoryStreamer {
public:
    struct Params {
        double horizon_length = 15.0; // Next 15 meters
        double dt = 0.1;              // 10Hz update
    };

    explicit TrajectoryStreamer(const Params& params);

    /**
     * @brief Update the current reference trajectory.
     */
    void setBaseTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj);

    /**
     * @brief Get the local horizon path based on current robot position.
     * @param current_pose Current robot pose in map frame.
     * @return A path containing points for the next N meters.
     */
    nav_msgs::msg::Path getLocalHorizon(const geometry_msgs::msg::Pose& current_pose);

private:
    Params params_;
    norlab_controllers_msgs::msg::PathSequence base_trajectory_;
    mutable std::mutex traj_mutex_;

    size_t findClosestPointIndex(const geometry_msgs::msg::Pose& pose);
};

} // namespace wiln
