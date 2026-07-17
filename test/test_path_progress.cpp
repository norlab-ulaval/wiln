#include <gtest/gtest.h>

#include <vector>

#include "wiln/PathProgress.hpp"

namespace {

std::vector<geometry_msgs::msg::PoseStamped> straightPath(
    int count,
    double spacing,
    bool reverse = false)
{
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = reverse
            ? static_cast<double>(count - 1 - i) * spacing
            : static_cast<double>(i) * spacing;
        pose.pose.orientation.w = 1.0;
        poses.push_back(pose);
    }
    return poses;
}

geometry_msgs::msg::Pose robotAt(double x, double y)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    return pose;
}

TEST(PathProgress, SkipsMissedWaypointAndKeepsTargetAhead)
{
    const auto path = straightPath(101, 0.1);
    const auto progress = wiln::selectPathProgress(
        path, 10, robotAt(2.62, 0.55), 1.0, 50);

    EXPECT_GE(progress.nearest_index, 25);
    EXPECT_LE(progress.nearest_index, 27);
    EXPECT_GE(progress.target_index, progress.nearest_index + 9);
    EXPECT_LE(progress.target_index, progress.nearest_index + 11);
    EXPECT_NEAR(progress.nearest_distance_m, 0.55, 0.02);
}

TEST(PathProgress, ProgressNeverMovesBackward)
{
    const auto path = straightPath(101, 0.1);
    const auto progress = wiln::selectPathProgress(
        path, 40, robotAt(2.0, 0.0), 1.0, 50);

    EXPECT_EQ(progress.nearest_index, 40);
    EXPECT_EQ(progress.target_index, 50);
}

TEST(PathProgress, WorksForReverseOrderedTrajectory)
{
    const auto path = straightPath(101, 0.1, true);
    const auto progress = wiln::selectPathProgress(
        path, 5, robotAt(7.35, -0.4), 0.8, 50);

    EXPECT_GE(progress.nearest_index, 25);
    EXPECT_LE(progress.nearest_index, 27);
    EXPECT_GT(progress.target_index, progress.nearest_index);
    EXPECT_NEAR(progress.nearest_distance_m, 0.4, 0.02);
}

TEST(PathProgress, SearchWindowPreventsDistantBranchJump)
{
    auto path = straightPath(121, 0.1);
    // A later branch passes through the robot position, but lies outside the
    // bounded forward window and must not steal progress at a route crossing.
    path[100].pose.position.x = 2.0;
    path[100].pose.position.y = 0.0;
    const auto progress = wiln::selectPathProgress(
        path, 15, robotAt(2.0, 0.1), 0.8, 30);

    EXPECT_LT(progress.nearest_index, 46);
    EXPECT_NE(progress.nearest_index, 100);
}

TEST(PathProgress, ClampsAtEndWithoutInvalidIndex)
{
    const auto path = straightPath(11, 0.1);
    const auto progress = wiln::selectPathProgress(
        path, 9, robotAt(1.2, 0.0), 3.0, 50);

    EXPECT_EQ(progress.nearest_index, 10);
    EXPECT_EQ(progress.target_index, 10);
}

}  // namespace
