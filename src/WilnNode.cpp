#include "wiln/WilnNode.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <filesystem>
#include <fstream>

namespace wiln {

WilnNode::WilnNode() : Node("wiln_node") {
    // Parameters
    double trajectory_speed = this->declare_parameter("trajectory_speed", 0.45);
    int smoothing_window = this->declare_parameter("smoothing_window_size", 15);
    double kappa_max = this->declare_parameter("kappa_max", 0.7);

    // Initialize Components
    TeachRecorder::Params teach_params;
    teach_params.smoothing_window = smoothing_window;
    recorder_ = std::make_unique<TeachRecorder>(teach_params);

    TrajectoryStreamer::Params stream_params;
    streamer_ = std::make_unique<TrajectoryStreamer>(stream_params);

    PathDeformer::Params deform_params;
    deform_params.kappa_max = kappa_max;
    deformer_ = std::make_unique<PathDeformer>(deform_params);

    // ROS 2 Interfaces
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    local_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_plan", 10);
    global_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory", qos);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_in", 10, std::bind(&WilnNode::onOdom, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose_in", 10, std::bind(&WilnNode::onPose, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/merged_points_filtered", 10, std::bind(&WilnNode::onScan, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel_in", 10, std::bind(&WilnNode::onCmdVel, this, std::placeholders::_1));

    start_recording_srv_ = this->create_service<std_srvs::srv::Empty>(
        "start_recording", std::bind(&WilnNode::handleStartRecording, this, std::placeholders::_1, std::placeholders::_2));
    stop_recording_srv_ = this->create_service<std_srvs::srv::Empty>(
        "stop_recording", std::bind(&WilnNode::handleStopRecording, this, std::placeholders::_1, std::placeholders::_2));
    play_line_srv_ = this->create_service<std_srvs::srv::Empty>(
        "play_line", std::bind(&WilnNode::handlePlayLine, this, std::placeholders::_1, std::placeholders::_2));
    save_ltr_srv_ = this->create_service<wiln::srv::SaveMapTraj>(
        "save_map_traj", std::bind(&WilnNode::handleSaveLTR, this, std::placeholders::_1, std::placeholders::_2));
    load_ltr_srv_ = this->create_service<wiln::srv::LoadMapTraj>(
        "load_map_traj", std::bind(&WilnNode::handleLoadLTR, this, std::placeholders::_1, std::placeholders::_2));

    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");

    // Streaming Timer (10Hz)
    stream_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&WilnNode::streamLoop, this));

    RCLCPP_INFO(this->get_logger(), "Modern WILN Node initialized.");
}

void WilnNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = msg->pose.pose;
}

void WilnNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (recorder_->isRecording()) {
        recorder_->addPose(*msg, driving_forward_);
    }
}

void WilnNode::onScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<Eigen::Vector3d> obstacles;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        obstacles.emplace_back(*iter_x, *iter_y, *iter_z);
    }
    
    std::lock_guard<std::mutex> lock(obs_mutex_);
    latest_obstacles_ = std::move(obstacles);
}

void WilnNode::onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    driving_forward_ = msg->twist.linear.x >= 0;
}

void WilnNode::streamLoop() {
    if (!playing_) return;

    geometry_msgs::msg::Pose pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose = current_pose_;
    }

    auto local_horizon = streamer_->getLocalHorizon(pose);
    
    std::vector<Eigen::Vector3d> obstacles;
    {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        obstacles = latest_obstacles_;
    }

    // Apply Elastic Band Deformation
    auto safe_plan = deformer_->deform(local_horizon, obstacles);
    
    local_plan_pub_->publish(safe_plan);
    
    // TODO: Send this safe_plan to mtt_path_follower via topic or action update
}

void WilnNode::handleStartRecording(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) {
    recorder_->start();
    RCLCPP_INFO(this->get_logger(), "Recording started.");
}

void WilnNode::handleStopRecording(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) {
    recorder_->stop();
    recorder_->smooth();
    RCLCPP_INFO(this->get_logger(), "Recording stopped and smoothed.");
}

void WilnNode::handlePlayLine(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) {
    auto traj = recorder_->getTrajectory();
    if (traj.paths.empty()) {
        RCLCPP_WARN(this->get_logger(), "No trajectory to play.");
        return;
    }
    streamer_->setBaseTrajectory(traj);
    playing_ = true;
    RCLCPP_INFO(this->get_logger(), "Playback started.");
}

void WilnNode::handleSaveLTR(const std::shared_ptr<wiln::srv::SaveMapTraj::Request> req, std::shared_ptr<wiln::srv::SaveMapTraj::Response>) {
    // Simplified save logic for brevity (can be expanded to full VTK + LTR)
    std::ofstream ltr(req->file_name.data);
    auto traj = recorder_->getTrajectory();
    for (const auto& path : traj.paths) {
        for (const auto& p : path.poses) {
            ltr << p.pose.position.x << "," << p.pose.position.y << "\n";
        }
    }
    RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s", req->file_name.data.c_str());
}

void WilnNode::handleLoadLTR(const std::shared_ptr<wiln::srv::LoadMapTraj::Request> req, std::shared_ptr<wiln::srv::LoadMapTraj::Response>) {
    // Simplified load logic
    RCLCPP_INFO(this->get_logger(), "Loading trajectory from %s", req->file_name.data.c_str());
    // ... logic to read file and populate streamer
}

} // namespace wiln

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wiln::WilnNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
