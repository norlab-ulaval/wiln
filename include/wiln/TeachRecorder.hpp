#pragma once

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
        int smoothing_window = 15;
    };

    explicit TeachRecorder(const Params& params);

    void start();
    void stop();
    void clear();
    bool isRecording() const { return recording_; }

    /**
     * @brief Feed a new pose from SLAM.
     * @param pose The current robot pose in the map frame.
     * @param forward Current driving direction.
     */
    void addPose(const geometry_msgs::msg::PoseStamped& pose, bool forward);

    norlab_controllers_msgs::msg::PathSequence getTrajectory() const;
    
    /**
     * @brief Apply a B-Spline or advanced smoothing to the current trajectory.
     */
    void smooth();

private:
    Params params_;
    bool recording_ = false;
    mutable std::mutex data_mutex_;
    norlab_controllers_msgs::msg::PathSequence trajectory_;
    bool last_forward_direction_ = true;

    double computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
};

} // namespace wiln
