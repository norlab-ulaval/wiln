#pragma once

#include <cstdint>
#include <vector>
#include <mutex>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wiln {

/**
 * @brief Handles the 'Teach' phase: recording, filtering, and smoothing.
 */
class TeachRecorder {
public:
    struct Params {
        double min_dist_between_poses = 0.05;
        double min_angle_between_poses = 0.5;
        double max_record_jump_m = 1.0;
        double max_record_yaw_jump_rad = 0.8;
        int smoothing_window = 15;
    };

    explicit TeachRecorder(const Params& params);

    void start();
    void stop();
    void clear();
    bool isRecording() const { return recording_; }
    uint32_t rejectedJumps() const { return rejected_jumps_; }

    /**
     * @brief Feed a new pose from SLAM.
     * @param pose The current robot pose in the map frame.
     * @param forward Current driving direction.
     */
    void addPose(const geometry_msgs::msg::PoseStamped& pose, bool forward);

    norlab_controllers_msgs::msg::PathSequence getTrajectory() const;

    /**
     * @brief Load an externally-parsed trajectory (e.g. from LTR file).
     */
    void setTrajectory(const norlab_controllers_msgs::msg::PathSequence& traj);

    /**
     * @brief Apply a B-Spline or advanced smoothing to the current trajectory.
     * @return true if smoothing was kept, false if it was rejected and reverted.
     */
    bool smooth();

private:
    Params params_;
    bool recording_ = false;
    uint32_t rejected_jumps_ = 0;
    mutable std::mutex data_mutex_;
    norlab_controllers_msgs::msg::PathSequence trajectory_;
    bool last_forward_direction_ = true;

    // 11A: Gap recovery — reconnect trajectory after ICP resumes following a dropout.
    // When poses jump > max_record_jump_m, we track whether ICP has stabilised at a
    // new consistent location. After gap_recovery_needed_ consecutive stable poses
    // (each within 3 × min_dist of the anchor), we start a new segment and reconnect.
    uint32_t gap_recovery_count_ = 0;
    geometry_msgs::msg::PoseStamped gap_recovery_anchor_;

    double computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& q) const;
    bool trajectoryHasLargeJump(const norlab_controllers_msgs::msg::PathSequence& trajectory) const;
};

} // namespace wiln
