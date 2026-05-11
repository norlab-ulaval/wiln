#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "wiln/TeachRecorder.hpp"
#include "wiln/TrajectoryStreamer.hpp"
#include "wiln/PathDeformer.hpp"
#include "wiln/TrajectoryManager.hpp"
#include "wiln/ObstacleManager.hpp"
#include "wiln/RobotModel.hpp"
#include "wiln/srv/save_map_traj.hpp"
#include "wiln/srv/load_map_traj.hpp"
#include "wiln/srv/play_loop.hpp"

#include <norlab_controllers_msgs/action/follow_path.hpp>
#include <norlab_controllers_msgs/msg/follower_options.hpp>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>
#include <norlab_icp_mapper_ros/srv/load_map.hpp>

#include <Eigen/Dense>

namespace wiln {

// ---------------------------------------------------------------------------
// Node state machine
// ---------------------------------------------------------------------------
enum class WilnState { IDLE, RECORDING, PLAYING, LOADING, SAVING };
inline const char* toString(WilnState s) {
    switch (s) {
        case WilnState::IDLE:      return "IDLE";
        case WilnState::RECORDING: return "RECORDING";
        case WilnState::PLAYING:   return "PLAYING";
        case WilnState::LOADING:   return "LOADING";
        case WilnState::SAVING:    return "SAVING";
    }
    return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// WilnNode
// ---------------------------------------------------------------------------
class WilnNode : public rclcpp::Node {
public:
    using FollowPath = norlab_controllers_msgs::action::FollowPath;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
    using SaveMap    = norlab_icp_mapper_ros::srv::SaveMap;
    using LoadMap    = norlab_icp_mapper_ros::srv::LoadMap;

    WilnNode();

private:
    // ------------------------------------------------------------------
    // Core components
    // ------------------------------------------------------------------
    std::unique_ptr<TeachRecorder>      recorder_;
    std::unique_ptr<TrajectoryStreamer> streamer_;
    std::unique_ptr<PathDeformer>       deformer_;
    std::unique_ptr<ObstacleManager>    obstacles_;
    std::shared_ptr<RobotModel>         robot_model_;

    // ------------------------------------------------------------------
    // TF2
    // ------------------------------------------------------------------
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ------------------------------------------------------------------
    // Callback groups
    // ------------------------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr services_group_;
    rclcpp::CallbackGroup::SharedPtr stream_group_;

    // ------------------------------------------------------------------
    // Publishers
    // ------------------------------------------------------------------
    // Transient-local: WILN recorded / loaded route (flattened nav_msgs/Path)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              global_plan_pub_;

    // *** REAL CONTROL TOPIC — consumed by mtt_path_follower for live steering ***
    // Topic name is parametrised via control_local_plan_topic (default /wiln/control/local_plan).
    // This is NOT diagnostic-only: the path follower prioritises it over the global plan.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              local_plan_pub_;

    // Debug / Foxglove visualisation — NOT consumed by the controller
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              horizon_debug_pub_;   // raw undeformed horizon
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              deformed_plan_pub_;   // elastic-band result

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub_;

    // ------------------------------------------------------------------
    // Subscriptions
    // ------------------------------------------------------------------
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           articulation_sub_;

    // ------------------------------------------------------------------
    // Services
    // ------------------------------------------------------------------
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr        start_recording_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr        stop_recording_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr        play_line_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr        cancel_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr        clear_trajectory_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr        smooth_trajectory_srv_;
    rclcpp::Service<wiln::srv::SaveMapTraj>::SharedPtr      save_ltr_srv_;
    rclcpp::Service<wiln::srv::LoadMapTraj>::SharedPtr      load_ltr_srv_;

    // ------------------------------------------------------------------
    // Action / service clients
    // ------------------------------------------------------------------
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    GoalHandle::SharedPtr                        active_goal_handle_;
    std::mutex                                   goal_handle_mutex_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr enable_mapping_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr disable_mapping_client_;
    rclcpp::Client<SaveMap>::SharedPtr              save_map_client_;
    rclcpp::Client<LoadMap>::SharedPtr              load_map_client_;

    // ------------------------------------------------------------------
    // Hot-path state (lock-free pointer swap)
    // ------------------------------------------------------------------
    std::atomic<WilnState> state_{WilnState::IDLE};
    std::atomic<bool>      driving_forward_{true};
    double                 trajectory_speed_{0.3};

    std::shared_ptr<const geometry_msgs::msg::Pose> current_pose_ptr_;
    std::mutex                                       pose_ptr_mutex_;

    // Fallback: last valid deformed plan (returned when deformation fails/times out)
    nav_msgs::msg::Path last_valid_plan_;
    std::mutex          last_valid_plan_mutex_;

    // ------------------------------------------------------------------
    // Timer
    // ------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr stream_timer_;

    // ------------------------------------------------------------------
    // Callbacks
    // ------------------------------------------------------------------
    void onOdom(nav_msgs::msg::Odometry::SharedPtr msg);
    void onPose(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onCmdVel(geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void onArticulationAngle(std_msgs::msg::Float64::SharedPtr msg);

    void streamLoop();

    // ------------------------------------------------------------------
    // Service handlers
    // ------------------------------------------------------------------
    void handleStartRecording(std::shared_ptr<std_srvs::srv::Empty::Request>,
                              std::shared_ptr<std_srvs::srv::Empty::Response>);
    void handleStopRecording(std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>);
    void handlePlayLine(std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>);
    void handleCancel(std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>);
    void handleClearTrajectory(std::shared_ptr<std_srvs::srv::Empty::Request>,
                               std::shared_ptr<std_srvs::srv::Empty::Response>);
    void handleSmoothTrajectory(std::shared_ptr<std_srvs::srv::Empty::Request>,
                                std::shared_ptr<std_srvs::srv::Empty::Response>);
    void handleSaveLTR(std::shared_ptr<wiln::srv::SaveMapTraj::Request>,
                       std::shared_ptr<wiln::srv::SaveMapTraj::Response>);
    void handleLoadLTR(std::shared_ptr<wiln::srv::LoadMapTraj::Request>,
                       std::shared_ptr<wiln::srv::LoadMapTraj::Response>);

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    void publishGlobalPlan(const norlab_controllers_msgs::msg::PathSequence& traj);
    void publishVisualization(const nav_msgs::msg::Path&          horizon,
                              const nav_msgs::msg::Path&          deformed,
                              const std::vector<Eigen::Vector3d>& obstacles,
                              const TrajectoryStreamer::Diag&     stream_diag,
                              const PathDeformer::Diag&           deform_diag);

    norlab_controllers_msgs::msg::PathSequence reverseTrajectory(
        const norlab_controllers_msgs::msg::PathSequence& traj) const;

    double distanceToPose(const geometry_msgs::msg::Pose&      from,
                          const geometry_msgs::msg::PoseStamped& to) const;
};

} // namespace wiln
