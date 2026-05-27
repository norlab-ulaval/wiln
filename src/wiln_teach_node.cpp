#include "wiln/WilnTeachNode.hpp"

namespace wiln {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
WilnTeachNode::WilnTeachNode() : Node("wiln_teach_node")
{
    // --- Parameters ---
    declare_parameter<int>("smoothing_window_size",      15);
    declare_parameter<double>("min_dist_between_poses",  0.05);
    declare_parameter<double>("min_angle_between_poses", 0.5);
    declare_parameter<double>("max_record_jump_m", 1.0);
    declare_parameter<double>("max_record_yaw_jump_rad", 0.8);
    const std::string odom_topic = declare_parameter("odom_topic", std::string("odom_in"));
    const std::string cmd_vel_topic = declare_parameter("cmd_vel_topic", std::string("cmd_vel_in"));
    const std::string command_topic = declare_parameter("command_topic", std::string("/wiln/command"));
    const std::string trajectory_topic = declare_parameter("trajectory_topic", std::string("/wiln/trajectory"));
    const std::string global_plan_topic = declare_parameter("global_plan_topic", std::string("/wiln/global_plan"));
    const std::string state_topic = declare_parameter("state_topic", std::string("/wiln/teach/state"));

    TeachRecorder::Params params;
    params.smoothing_window      = get_parameter("smoothing_window_size").as_int();
    params.min_dist_between_poses  = get_parameter("min_dist_between_poses").as_double();
    params.min_angle_between_poses = get_parameter("min_angle_between_poses").as_double();
    params.max_record_jump_m = get_parameter("max_record_jump_m").as_double();
    params.max_record_yaw_jump_rad = get_parameter("max_record_yaw_jump_rad").as_double();
    recorder_ = std::make_unique<TeachRecorder>(params);

    // --- QoS ---
    auto be_qos        = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    auto cmd_qos       = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    // --- Subscriptions ---
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, be_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic, be_qos,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) { onCmdVel(msg); });

    command_sub_ = create_subscription<std_msgs::msg::String>(
        command_topic, cmd_qos,
        [this](std_msgs::msg::String::SharedPtr msg) { onCommand(msg); });

    // --- Publishers ---
    trajectory_pub_  = create_publisher<norlab_controllers_msgs::msg::PathSequence>(
        trajectory_topic, transient_qos);
    global_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
        global_plan_topic, transient_qos);
    state_pub_ = create_publisher<wiln::msg::WilnState>(
        state_topic, transient_qos);

    publishState(wiln::msg::WilnState::IDLE, "ready");
    RCLCPP_INFO(get_logger(),
        "wiln_teach_node started. odom=%s cmd_vel=%s command=%s trajectory=%s smoothing_window=%d, min_dist=%.2f m",
        odom_topic.c_str(), cmd_vel_topic.c_str(), command_topic.c_str(), trajectory_topic.c_str(),
        params.smoothing_window, params.min_dist_between_poses);
}

// ---------------------------------------------------------------------------
// Odometry callback -- feed recorder when RECORDING
// ---------------------------------------------------------------------------
void WilnTeachNode::onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (state_.load() != State::RECORDING) return;

    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose   = msg->pose.pose;
    recorder_->addPose(ps, driving_forward_.load());
}

// ---------------------------------------------------------------------------
// cmd_vel callback -- track driving direction
// ---------------------------------------------------------------------------
void WilnTeachNode::onCmdVel(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    driving_forward_.store(msg->twist.linear.x >= 0.0);
}

// ---------------------------------------------------------------------------
// Command dispatcher
// ---------------------------------------------------------------------------
void WilnTeachNode::onCommand(std_msgs::msg::String::SharedPtr msg)
{
    const std::string& cmd = msg->data;
    if (cmd == "start_recording")   handleStartRecording();
    else if (cmd == "stop_recording")    handleStopRecording();
    else if (cmd == "smooth_trajectory") handleSmoothTrajectory();
    else if (cmd == "clear_trajectory")  handleClearTrajectory();
}

// ---------------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------------
void WilnTeachNode::handleStartRecording()
{
    State expected = State::IDLE;
    if (!state_.compare_exchange_strong(expected, State::RECORDING)) {
        RCLCPP_WARN(get_logger(), "Cannot start recording — not IDLE (current: %d)",
            static_cast<int>(state_.load()));
        return;
    }
    recorder_->start();
    publishState(wiln::msg::WilnState::RECORDING, "recording");
    RCLCPP_INFO(get_logger(), "Recording started.");
}

void WilnTeachNode::handleStopRecording()
{
    if (state_.load() != State::RECORDING) {
        RCLCPP_WARN(get_logger(), "Not recording — ignoring stop_recording.");
        return;
    }
    recorder_->stop();
    const bool smoothing_kept = recorder_->smooth();

    auto traj = recorder_->getTrajectory();
    state_.store(State::IDLE);

    size_t total = 0;
    for (const auto& p : traj.paths) total += p.poses.size();
    if (smoothing_kept) {
        RCLCPP_INFO(get_logger(),
            "Recording stopped and smoothed: %zu segment(s), %zu poses, %u rejected ICP jump(s).",
            traj.paths.size(), total, recorder_->rejectedJumps());
    } else {
        RCLCPP_WARN(get_logger(),
            "Recording stopped: smoothing rejected because it introduced a large route jump. "
            "Kept raw filtered trajectory: %zu segment(s), %zu poses, %u rejected ICP jump(s).",
            traj.paths.size(), total, recorder_->rejectedJumps());
    }

    publishTrajectory(traj);
    publishState(
        wiln::msg::WilnState::IDLE,
        std::string("trajectory recorded; rejected_jumps=") +
            std::to_string(recorder_->rejectedJumps()) +
            (smoothing_kept ? "; smoothing=kept" : "; smoothing=reverted"));
}

void WilnTeachNode::handleSmoothTrajectory()
{
    if (state_.load() != State::IDLE) {
        RCLCPP_WARN(get_logger(), "Cannot smooth while recording.");
        return;
    }
    const bool smoothing_kept = recorder_->smooth();
    auto traj = recorder_->getTrajectory();
    publishTrajectory(traj);
    if (smoothing_kept) {
        RCLCPP_INFO(get_logger(), "Trajectory re-smoothed.");
    } else {
        RCLCPP_WARN(get_logger(), "Trajectory smoothing rejected because it introduced a large route jump.");
    }
}

void WilnTeachNode::handleClearTrajectory()
{
    if (state_.load() == State::RECORDING) {
        RCLCPP_WARN(get_logger(), "Cannot clear while recording.");
        return;
    }
    recorder_->clear();
    publishState(wiln::msg::WilnState::IDLE, "trajectory cleared");
    RCLCPP_INFO(get_logger(), "Trajectory cleared.");
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void WilnTeachNode::publishTrajectory(
    const norlab_controllers_msgs::msg::PathSequence& traj)
{
    trajectory_pub_->publish(traj);

    // Flatten to nav_msgs::Path for Foxglove/RViz visualisation
    nav_msgs::msg::Path global;
    global.header = traj.header;
    global.header.stamp = now();
    for (const auto& path : traj.paths)
        for (const auto& ps : path.poses)
            global.poses.push_back(ps);
    global_plan_pub_->publish(global);
}

void WilnTeachNode::publishState(uint8_t state_code, const std::string& detail)
{
    wiln::msg::WilnState msg;
    msg.stamp  = now();
    msg.state  = state_code;
    msg.detail = detail;

    if (recorder_) {
        const auto traj = recorder_->getTrajectory();
        size_t total = 0;
        for (const auto& p : traj.paths) total += p.poses.size();
        msg.trajectory_poses = static_cast<uint32_t>(total);
    }
    state_pub_->publish(msg);
}

} // namespace wiln

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wiln::WilnTeachNode>());
    rclcpp::shutdown();
    return 0;
}
