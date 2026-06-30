#include "wiln/WilnObstacleNode.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace wiln {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WilnObstacleNode::WilnObstacleNode() : Node("wiln_obstacle_node")
{
    // Callback groups: LiDAR subs are Reentrant (may block on TF, must not
    // starve odom/timer); odom + publish timer are MutuallyExclusive.
    lidar_cb_group_   = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // --- Declare parameters ---
    declare_parameter<std::vector<std::string>>("lidar_topics",
        std::vector<std::string>{"/merged_points_filtered"});
    declare_parameter<std::string>("target_frame",        "map");
    declare_parameter<double>("obstacle_max_age_s",        0.5);
    declare_parameter<double>("crop_length",              25.0);
    declare_parameter<double>("crop_width",                8.0);
    declare_parameter<double>("crop_x_min",              -12.5);
    declare_parameter<double>("crop_x_max",               12.5);
    declare_parameter<double>("crop_y_abs",                8.0);
    declare_parameter<double>("crop_z_min",               -0.3);
    declare_parameter<double>("crop_z_max",                2.0);
    declare_parameter<double>("voxel_size",                0.15);
    declare_parameter<double>("min_range",                 0.5);
    declare_parameter<double>("max_range",                20.0);
    declare_parameter<bool>("enable_self_filter",         false);
    declare_parameter<std::vector<double>>("self_filter_chassis_box", std::vector<double>{});
    declare_parameter<std::vector<double>>("self_filter_lidar_cage_box", std::vector<double>{});
    declare_parameter<std::vector<double>>("self_filter_rsairy_box", std::vector<double>{});
    declare_parameter<std::vector<double>>("self_filter_trailer_box", std::vector<double>{});
    declare_parameter<double>("publish_rate_hz",          10.0);
    const std::string odom_topic = declare_parameter("odom_topic", std::string("odom_in"));
    const std::string obstacles_topic = declare_parameter("obstacles_topic", std::string("/wiln/obstacles"));

    // --- Read parameters into ObstacleParams ---
    ObstacleParams params;
    params.lidar_topics      = get_parameter("lidar_topics").as_string_array();
    params.target_frame      = get_parameter("target_frame").as_string();
    params.obstacle_max_age_s = get_parameter("obstacle_max_age_s").as_double();
    params.crop_length       = get_parameter("crop_length").as_double();
    params.crop_width        = get_parameter("crop_width").as_double();
    params.crop_x_min        = get_parameter("crop_x_min").as_double();
    params.crop_x_max        = get_parameter("crop_x_max").as_double();
    params.crop_y_abs        = get_parameter("crop_y_abs").as_double();
    params.crop_z_min        = get_parameter("crop_z_min").as_double();
    params.crop_z_max        = get_parameter("crop_z_max").as_double();
    params.voxel_size        = get_parameter("voxel_size").as_double();
    params.min_range         = get_parameter("min_range").as_double();
    params.max_range         = get_parameter("max_range").as_double();
    params.enable_self_filter = get_parameter("enable_self_filter").as_bool();
    double publish_rate_hz   = get_parameter("publish_rate_hz").as_double();

    auto read_box = [this, &params](const std::string& param_name, const std::string& box_name) {
        const auto values = get_parameter(param_name).as_double_array();
        if (values.empty()) return;
        if (values.size() != 6) {
            RCLCPP_WARN(get_logger(),
                "Ignoring %s: expected [x_min, x_max, y_min, y_max, z_min, z_max], got %zu values.",
                param_name.c_str(), values.size());
            return;
        }
        params.self_filter_boxes.push_back(ObstacleParams::SelfFilterBox{
            box_name,
            values[0], values[1],
            values[2], values[3],
            values[4], values[5]
        });
    };
    read_box("self_filter_chassis_box", "chassis");
    read_box("self_filter_lidar_cage_box", "lidar_cage");
    read_box("self_filter_rsairy_box", "rsairy");
    read_box("self_filter_trailer_box", "trailer");

    target_frame_ = params.target_frame;

    // --- TF ---
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Obstacle manager (subscribes to lidar topics on lidar_cb_group_) ---
    obstacle_manager_ = std::make_unique<ObstacleManager>(this, params, tf_buffer_, lidar_cb_group_);

    // --- Odometry subscription (control group — not blocked by LiDAR TF waits) ---
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    rclcpp::SubscriptionOptions ctrl_opts;
    ctrl_opts.callback_group = control_cb_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); },
        ctrl_opts);

    // --- Obstacle cloud publisher ---
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic, pub_qos);

    // --- Publish timer (control group — guaranteed 10 Hz regardless of LiDAR load) ---
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer_ = create_wall_timer(period_ns, [this]() { publishObstacles(); },
        control_cb_group_);

    RCLCPP_INFO(get_logger(),
        "wiln_obstacle_node started. odom=%s obstacles=%s %zu lidar topic(s), frame='%s', rate=%.0f Hz, crop=[x %.2f..%.2f y +/-%.2f z %.2f..%.2f], self_filter=%s (%zu boxes).",
        odom_topic.c_str(), obstacles_topic.c_str(), params.lidar_topics.size(), target_frame_.c_str(), publish_rate_hz,
        params.crop_x_min, params.crop_x_max, params.crop_y_abs, params.crop_z_min, params.crop_z_max,
        params.enable_self_filter ? "true" : "false", params.self_filter_boxes.size());
}

// ---------------------------------------------------------------------------
// Odom callback -- update robot pose
// ---------------------------------------------------------------------------
void WilnObstacleNode::onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_  = msg->pose.pose;
    pose_received_ = true;
}

// ---------------------------------------------------------------------------
// Timer callback -- process obstacles and publish
// ---------------------------------------------------------------------------
void WilnObstacleNode::publishObstacles()
{
    geometry_msgs::msg::Pose pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!pose_received_) return;
        pose = current_pose_;
    }

    ObstacleManager::Diag diag;
    auto snapshot = obstacle_manager_->getSnapshot(pose, &diag);
    if (!snapshot || snapshot->empty()) return;

    auto cloud = toPointCloud2(*snapshot, target_frame_, now());
    obstacles_pub_->publish(cloud);

    RCLCPP_DEBUG(get_logger(),
        "Published %zu obstacle points (raw=%zu, active_topics=%zu, merge=%.1f ms).",
        snapshot->size(), diag.raw_points, diag.active_topics.size(), diag.merge_time_ms);
}

// ---------------------------------------------------------------------------
// Static helper -- convert Eigen points to PointCloud2
// ---------------------------------------------------------------------------
sensor_msgs::msg::PointCloud2 WilnObstacleNode::toPointCloud2(
    const std::vector<Eigen::Vector3d>& points,
    const std::string&                  frame_id,
    const rclcpp::Time&                 stamp)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;
    msg.height          = 1;
    msg.width           = static_cast<uint32_t>(points.size());
    msg.is_bigendian    = false;
    msg.is_dense        = true;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");

    for (const auto& p : points) {
        *ix = static_cast<float>(p.x());
        *iy = static_cast<float>(p.y());
        *iz = static_cast<float>(p.z());
        ++ix; ++iy; ++iz;
    }

    return msg;
}

} // namespace wiln

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wiln::WilnObstacleNode>();

    // 3-thread executor:
    //   Thread 1–2 : Reentrant lidar_cb_group_ — up to 2 LiDAR callbacks in
    //                parallel (TF lookups per-scan do not block each other).
    //   Thread 3   : MutuallyExclusive control_cb_group_ — odom + publish
    //                timer run sequentially, guaranteed 10 Hz cadence even
    //                when LiDAR TF is slow (bag replay, startup).
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 3);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
