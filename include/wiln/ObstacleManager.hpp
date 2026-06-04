#pragma once
/**
 * ObstacleManager.hpp — Multi-LiDAR obstacle aggregation for WILN.
 *
 * Responsibilities:
 *   - Subscribe to N configurable PointCloud2 topics.
 *   - Transform each cloud into target_frame via TF2.
 *   - Discard stale clouds (age > obstacle_max_age_s).
 *   - Voxel-downsample using a compact hash-grid.
 *   - Crop to a box around the robot's current pose (not the whole map).
 *   - Height-filter to keep only relevant obstacles.
 *   - Expose a thread-safe snapshot for the 10 Hz stream loop.
 *
 * The snapshot is rebuilt asynchronously as clouds arrive; the stream loop
 * always reads the last valid snapshot without blocking.
 */

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace wiln {

struct ObstacleParams {
    struct SelfFilterBox {
        std::string name;
        double x_min = 0.0;
        double x_max = 0.0;
        double y_min = 0.0;
        double y_max = 0.0;
        double z_min = 0.0;
        double z_max = 0.0;
    };

    std::vector<std::string> lidar_topics = {"/merged_points_filtered"};
    std::string target_frame  = "map";
    double obstacle_max_age_s = 0.5;   // clouds older than this are ignored
    double crop_length        = 25.0;  // [m] forward + backward from robot pose
    double crop_width         = 8.0;   // [m] lateral half-width
    double crop_x_min         = -12.5; // [m] local robot-frame rear bound
    double crop_x_max         =  12.5; // [m] local robot-frame forward bound
    double crop_y_abs         =   8.0; // [m] local robot-frame lateral half-width
    double crop_z_min         = -0.3;  // [m] local robot-frame height floor
    double crop_z_max         =  2.0;  // [m] local robot-frame height ceiling
    double voxel_size         = 0.15;  // [m] voxel grid leaf size
    double min_range          = 0.5;   // [m] ignore sensor self-return
    double max_range          = 20.0;  // [m] ignore far noise
    bool enable_self_filter   = false;
    std::vector<SelfFilterBox> self_filter_boxes;
};

// ---------------------------------------------------------------------------
class ObstacleManager {
public:
    struct Diag {
        size_t raw_points         = 0;
        size_t after_filter       = 0;
        int    stale_topics       = 0;
        double merge_time_ms      = 0.0;
        std::vector<std::string> active_topics;
    };

    ObstacleManager(rclcpp::Node*                      node,
                    const ObstacleParams&               params,
                    std::shared_ptr<tf2_ros::Buffer>    tf_buffer,
                    rclcpp::CallbackGroup::SharedPtr    cb_group = nullptr);

    /**
     * Returns the last merged+filtered obstacle snapshot cropped around
     * robot_pose.  Never blocks.  If no valid cloud is available, returns an
     * empty vector.  Optional diag output for Foxglove.
     */
    std::shared_ptr<const std::vector<Eigen::Vector3d>>
    getSnapshot(const geometry_msgs::msg::Pose& robot_pose,
                Diag* diag = nullptr);

private:
    // Per-source cache entry
    struct CloudEntry {
        rclcpp::Time stamp;
        std::vector<Eigen::Vector3d> points;  // already in target_frame
    };

    rclcpp::Node*                   node_;
    ObstacleParams                  params_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::map<std::string,
             rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;

    std::mutex                    cache_mutex_;
    std::map<std::string, CloudEntry> cloud_cache_;

    // Latest merged snapshot (swap on every update, read locklessly by stream loop)
    std::shared_ptr<const std::vector<Eigen::Vector3d>> latest_snapshot_;
    std::mutex snapshot_mutex_;

    // ---- helpers ----
    void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
                 const std::string& topic);

    /** Simple voxel-grid downsample: one representative point per cell. */
    static std::vector<Eigen::Vector3d> voxelDownsample(
        const std::vector<Eigen::Vector3d>& pts, double voxel_size);

    /** AABB crop + height + range filter in target_frame. */
    std::vector<Eigen::Vector3d> cropAndFilter(
        const std::vector<Eigen::Vector3d>& pts,
        const geometry_msgs::msg::Pose&     robot_pose) const;
};

} // namespace wiln
