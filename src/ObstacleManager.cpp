#include "wiln/ObstacleManager.hpp"

#include <chrono>
#include <tf2/exceptions.h>

namespace wiln {

namespace {
bool isInsideBox(const ObstacleParams::SelfFilterBox& box,
                 double x,
                 double y,
                 double z)
{
    return x >= box.x_min && x <= box.x_max &&
           y >= box.y_min && y <= box.y_max &&
           z >= box.z_min && z <= box.z_max;
}
} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
ObstacleManager::ObstacleManager(rclcpp::Node*                   node,
                                 const ObstacleParams&            params,
                                 std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                 rclcpp::CallbackGroup::SharedPtr cb_group)
    : node_(node), params_(params), tf_buffer_(tf_buffer)
{
    rclcpp::SubscriptionOptions opts;
    if (cb_group) opts.callback_group = cb_group;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    for (const auto& topic : params_.lidar_topics) {
        auto cb = [this, topic](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            onCloud(msg, topic);
        };
        subs_[topic] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, qos, cb, opts);

        RCLCPP_INFO(node_->get_logger(),
            "ObstacleManager subscribed to: %s", topic.c_str());
    }
}

// ---------------------------------------------------------------------------
// Public: snapshot for stream loop
// ---------------------------------------------------------------------------
std::shared_ptr<const std::vector<Eigen::Vector3d>>
ObstacleManager::getSnapshot(const geometry_msgs::msg::Pose& robot_pose, Diag* diag)
{
    auto t0 = std::chrono::steady_clock::now();
    rclcpp::Time now = node_->now();

    std::vector<Eigen::Vector3d> merged;
    merged.reserve(2048);

    int stale = 0;
    std::vector<std::string> active;

    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        for (auto& [topic, entry] : cloud_cache_) {
            double age = (now - entry.stamp).seconds();
            if (age > params_.obstacle_max_age_s) {
                ++stale;
                continue;
            }
            active.push_back(topic);
            merged.insert(merged.end(), entry.points.begin(), entry.points.end());
        }
    }

    if (diag) diag->raw_points   = merged.size();
    if (diag) diag->stale_topics = stale;

    // Crop + self filter in the robot-local frame. Points stay in target_frame
    // for WILN's path deformer; only the selection is local to the robot pose.
    auto cropped = cropAndFilter(merged, robot_pose);

    // Voxel downsample
    auto downsampled = voxelDownsample(cropped, params_.voxel_size);

    auto snapshot = std::make_shared<const std::vector<Eigen::Vector3d>>(
        std::move(downsampled));

    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        latest_snapshot_ = snapshot;
    }

    auto t1 = std::chrono::steady_clock::now();
    if (diag) {
        diag->after_filter  = snapshot->size();
        diag->merge_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        diag->active_topics = std::move(active);
    }

    return snapshot;
}

// ---------------------------------------------------------------------------
// Private: incoming cloud callback
// ---------------------------------------------------------------------------
void ObstacleManager::onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
                               const std::string& topic)
{
    // Transform into target_frame if needed
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->width * msg->height);

    bool needs_transform = (msg->header.frame_id != params_.target_frame);
    geometry_msgs::msg::TransformStamped tf_stamped;

    if (needs_transform && tf_buffer_) {
        // 9A: Use message timestamp for TF lookup (not TimePointZero).
        // At 1.5 m/s, using "latest" vs "at capture time" introduces up to
        // 15 cm spatial error per 100ms of TF delay — causing false positives
        // (phantom obstacles) or false negatives (real obstacles shifted out of
        // the crop box). Fallback to TimePointZero on exception (startup / replay).
        bool tf_ok = false;
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                params_.target_frame,
                msg->header.frame_id,
                rclcpp::Time(msg->header.stamp),
                rclcpp::Duration::from_seconds(0.1));
            tf_ok = true;
        } catch (const tf2::TransformException&) {}

        if (!tf_ok) {
            try {
                tf_stamped = tf_buffer_->lookupTransform(
                    params_.target_frame,
                    msg->header.frame_id,
                    tf2::TimePointZero);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                    "ObstacleManager: TF lookup failed for %s: %s",
                    topic.c_str(), ex.what());
                return;
            }
        }
    }

    // 9B: Precompute rotation matrix and translation outside the per-point loop.
    // The transform doesn't change between points — building Quaterniond per-point
    // at 30K pts/scan × 10 Hz was wasteful.
    Eigen::Matrix3d rot_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_vec = Eigen::Vector3d::Zero();
    if (needs_transform) {
        const auto& t = tf_stamped.transform.translation;
        const auto& q = tf_stamped.transform.rotation;
        rot_matrix = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
        translation_vec = Eigen::Vector3d(t.x, t.y, t.z);
    }

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;

        Eigen::Vector3d p(*ix, *iy, *iz);

        // Range pre-filter (fast, in sensor frame)
        double r2 = p.x() * p.x() + p.y() * p.y();
        if (r2 < params_.min_range * params_.min_range) continue;
        if (r2 > params_.max_range * params_.max_range) continue;

        if (needs_transform) {
            p = rot_matrix * p + translation_vec;
        }

        pts.push_back(p);
    }

    rclcpp::Time stamp(msg->header.stamp);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cloud_cache_[topic] = {stamp, std::move(pts)};
}

// ---------------------------------------------------------------------------
// Private: voxel downsample
// ---------------------------------------------------------------------------
struct VoxelKey {
    int64_t ix, iy, iz;
    bool operator==(const VoxelKey& o) const {
        return ix == o.ix && iy == o.iy && iz == o.iz;
    }
};
struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        size_t h = std::hash<int64_t>{}(k.ix);
        h ^= std::hash<int64_t>{}(k.iy) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= std::hash<int64_t>{}(k.iz) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};

std::vector<Eigen::Vector3d> ObstacleManager::voxelDownsample(
    const std::vector<Eigen::Vector3d>& pts, double voxel_size)
{
    if (pts.empty() || voxel_size <= 0.0) return pts;

    std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> grid;
    grid.reserve(pts.size() / 4);

    double inv = 1.0 / voxel_size;
    for (const auto& p : pts) {
        VoxelKey k{
            static_cast<int64_t>(std::floor(p.x() * inv)),
            static_cast<int64_t>(std::floor(p.y() * inv)),
            static_cast<int64_t>(std::floor(p.z() * inv))
        };
        // Keep the first point in each voxel (fast, deterministic)
        grid.emplace(k, p);
    }

    std::vector<Eigen::Vector3d> out;
    out.reserve(grid.size());
    for (auto& [k, v] : grid) out.push_back(v);
    return out;
}

// ---------------------------------------------------------------------------
// Private: crop AABB + height filter
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector3d> ObstacleManager::cropAndFilter(
    const std::vector<Eigen::Vector3d>& pts,
    const geometry_msgs::msg::Pose&     robot_pose) const
{
    if (pts.empty()) return {};

    // Robot yaw for oriented crop box
    const auto& q = robot_pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double ca = std::cos(yaw);
    double sa = std::sin(yaw);
    double rx = robot_pose.position.x;
    double ry = robot_pose.position.y;

    std::vector<Eigen::Vector3d> out;
    out.reserve(pts.size() / 4);

    for (const auto& p : pts) {
        // Rotate into robot frame. Roll/pitch are intentionally ignored here:
        // WILN obstacle gating needs a stable ground-plane crop around the MTT.
        double dx = p.x() - rx;
        double dy = p.y() - ry;
        double local_x =  ca * dx + sa * dy;
        double local_y = -sa * dx + ca * dy;
        double local_z = p.z() - robot_pose.position.z;

        if (local_x < params_.crop_x_min || local_x > params_.crop_x_max) continue;
        if (std::abs(local_y) > params_.crop_y_abs) continue;
        if (local_z < params_.crop_z_min || local_z > params_.crop_z_max) continue;

        if (params_.enable_self_filter) {
            bool is_self = false;
            for (const auto& box : params_.self_filter_boxes) {
                if (isInsideBox(box, local_x, local_y, local_z)) {
                    is_self = true;
                    break;
                }
            }
            if (is_self) continue;
        }

        out.push_back(p);
    }
    return out;
}

} // namespace wiln
