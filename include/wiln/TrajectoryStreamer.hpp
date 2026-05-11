#pragma once
/**
 * TrajectoryStreamer.hpp — Local-horizon extractor for the repeat phase.
 *
 * Key improvements over the naïve O(N) search:
 *   - Builds a flat index of all poses at setBaseTrajectory() time.
 *   - Tracks monotonic progress: searches only within
 *     [last_idx - search_window_backward, last_idx + search_window_forward].
 *   - Prevents trajectory-crossing jumps during loops or sharp turns.
 *   - Resets cleanly when a new trajectory is loaded.
 *   - Emits diagnostics for Foxglove (dist-to-path, heading error, …).
 */

#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>

namespace wiln {

class TrajectoryStreamer {
public:
    struct Params {
        double horizon_length         = 15.0;  // [m] lookahead distance
        size_t search_window_backward = 10;    // poses to look back (handle deceleration)
        size_t search_window_forward  = 80;    // poses to look forward
    };

    // Diagnostic output for Foxglove
    struct Diag {
        size_t closest_global_idx = 0;
        double dist_to_path_m     = 0.0;   // lateral deviation [m]
        double heading_error_rad  = 0.0;   // heading vs. path tangent [rad]
        size_t horizon_points     = 0;
        bool   full_search_used   = false; // true if window search failed → fell back to full
    };

    explicit TrajectoryStreamer(const Params& params);

    /** Load (or replace) the base trajectory and reset the progress tracker. */
    void setBaseTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj);

    /**
     * Extract the local horizon around current_pose.
     * @param diag  Optional diagnostics output.
     */
    nav_msgs::msg::Path getLocalHorizon(const geometry_msgs::msg::Pose& current_pose,
                                        Diag* diag = nullptr);

private:
    Params params_;
    mutable std::mutex traj_mutex_;

    // Flat view of the trajectory (rebuilt by setBaseTrajectory)
    struct FlatPose {
        geometry_msgs::msg::PoseStamped pose;
        size_t path_segment;   // which DirectionalPath this came from
    };
    std::vector<FlatPose>                              flat_index_;
    norlab_controllers_msgs::msg::PathSequence          base_trajectory_;
    std::string                                         frame_id_;

    // Monotonic progress state (protected by traj_mutex_)
    size_t last_closest_idx_ = 0;

    void buildFlatIndex(const norlab_controllers_msgs::msg::PathSequence& traj);

    size_t findClosestInWindow(const geometry_msgs::msg::Pose& pose,
                               bool* full_search_used) const;
};

} // namespace wiln
