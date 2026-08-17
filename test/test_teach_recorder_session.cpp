#include <cmath>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "wiln/TeachRecorder.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(orientation);
  return result;
}

}  // namespace

TEST(TeachRecorderSession, StartAlwaysReplacesPreviouslyLoadedTrajectory)
{
  wiln::TeachRecorder recorder(wiln::TeachRecorder::Params{});
  norlab_controllers_msgs::msg::PathSequence loaded;
  norlab_controllers_msgs::msg::DirectionalPath path;
  path.forward = true;
  path.poses.push_back(pose(10.0, 0.0, 0.0));
  path.poses.push_back(pose(11.0, 0.0, 0.0));
  loaded.paths.push_back(path);
  recorder.setTrajectory(loaded);

  recorder.start();
  const auto fresh = recorder.getTrajectory();
  EXPECT_TRUE(fresh.paths.empty());
}

TEST(TeachRecorderSession, ForwardCircleRemainsOneContinuousSegment)
{
  wiln::TeachRecorder::Params params;
  params.smoothing_window = 3;
  params.min_dist_between_poses = 0.05;
  params.min_angle_between_poses = 0.03;
  params.max_record_jump_m = 1.0;
  params.max_record_yaw_jump_rad = 0.8;
  params.resample_spacing_m = 0.05;
  wiln::TeachRecorder recorder(params);
  recorder.start();

  constexpr int samples = 100;
  constexpr double radius = 3.0;
  for (int i = 0; i <= samples; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / samples;
    recorder.addPose(
      pose(radius * std::cos(angle), radius * std::sin(angle), angle + M_PI_2),
      true);
  }
  recorder.stop();
  ASSERT_TRUE(recorder.smooth());

  const auto trajectory = recorder.getTrajectory();
  ASSERT_EQ(trajectory.paths.size(), 1U);
  ASSERT_GT(trajectory.paths.front().poses.size(), 100U);
  for (size_t i = 1; i < trajectory.paths.front().poses.size(); ++i) {
    const auto & previous = trajectory.paths.front().poses[i - 1].pose.position;
    const auto & current = trajectory.paths.front().poses[i].pose.position;
    EXPECT_LT(std::hypot(current.x - previous.x, current.y - previous.y), 0.2);
  }
}
