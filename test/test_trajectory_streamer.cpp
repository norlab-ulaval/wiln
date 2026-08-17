#include <gtest/gtest.h>

#include "wiln/TrajectoryStreamer.hpp"

namespace
{

norlab_controllers_msgs::msg::DirectionalPath segment(
  std::initializer_list<double> xs)
{
  norlab_controllers_msgs::msg::DirectionalPath path;
  path.forward = true;
  path.header.frame_id = "map";
  for (const double x : xs) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(TrajectoryStreamer, LocalHorizonNeverConcatenatesDirectionalSegments)
{
  norlab_controllers_msgs::msg::PathSequence trajectory;
  trajectory.header.frame_id = "map";
  trajectory.paths.push_back(segment({0.0, 0.5, 1.0}));
  trajectory.paths.push_back(segment({10.0, 10.5, 11.0}));

  wiln::TrajectoryStreamer streamer(wiln::TrajectoryStreamer::Params{});
  streamer.setBaseTrajectory(trajectory);
  geometry_msgs::msg::Pose robot;
  robot.orientation.w = 1.0;

  const auto horizon = streamer.getLocalHorizon(robot);
  ASSERT_EQ(horizon.poses.size(), 3U);
  EXPECT_DOUBLE_EQ(horizon.poses.back().pose.position.x, 1.0);
}

TEST(TrajectoryStreamer, NewTrajectoryReplacesPreviousOne)
{
  norlab_controllers_msgs::msg::PathSequence first;
  first.header.frame_id = "map";
  first.paths.push_back(segment({0.0, 1.0, 2.0}));
  norlab_controllers_msgs::msg::PathSequence second;
  second.header.frame_id = "map";
  second.paths.push_back(segment({20.0, 21.0, 22.0}));

  wiln::TrajectoryStreamer streamer(wiln::TrajectoryStreamer::Params{});
  streamer.setBaseTrajectory(first);
  streamer.setBaseTrajectory(second);
  geometry_msgs::msg::Pose robot;
  robot.position.x = 20.0;
  robot.orientation.w = 1.0;

  const auto horizon = streamer.getLocalHorizon(robot);
  ASSERT_FALSE(horizon.poses.empty());
  EXPECT_DOUBLE_EQ(horizon.poses.front().pose.position.x, 20.0);
  for (const auto & pose : horizon.poses) {
    EXPECT_GE(pose.pose.position.x, 20.0);
  }
}
