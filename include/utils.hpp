#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>

inline double wrapToPi(double angle) {
    return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

std::tuple<double, double, double> quatToRPY(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);
}

std::pair<float, float> diffBetweenPoses(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
{
    float dx = pose2.position.x - pose1.position.x;
    float dy = pose2.position.y - pose1.position.y;
    auto [roll1, pitch1, yaw1] = quatToRPY(pose1.orientation);
    auto [roll2, pitch2, yaw2] = quatToRPY(pose2.orientation);
    float diff_ang = wrapToPi(yaw2 - yaw1);
    float diff_lin = std::sqrt(dx * dx + dy * dy);
    return std::make_pair(diff_lin, diff_ang);
}

nav_msgs::msg::Path reversePath(const nav_msgs::msg::Path &path)
{
    nav_msgs::msg::Path reversedPath = path;
    std::reverse(reversedPath.poses.begin(), reversedPath.poses.end());
    return reversedPath;
}

geometry_msgs::msg::Quaternion flipQuaternion(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf2::Quaternion flippedQuat;
    flippedQuat.setRPY(-roll, -pitch, yaw + M_PI);
    return tf2::toMsg(flippedQuat);
}

nav_msgs::msg::Path flipPath(const nav_msgs::msg::Path &path)
{
    nav_msgs::msg::Path flippedPath = path;
    for (auto &pose : flippedPath.poses)
    {
        pose.pose.orientation = flipQuaternion(pose.pose.orientation);
    }
    return flippedPath;
}

nav_msgs::msg::Path smoothPathLowPass(const nav_msgs::msg::Path &roughPath, int windowSize)
{
    nav_msgs::msg::Path smoothPath(roughPath);

    // Check that window size is odd
    if (windowSize % 2 == 0)
    {
        windowSize++;
    }

    // Iterate through each pose in the roughPath
    for (size_t j = 0; j < roughPath.poses.size(); ++j)
    {
        double sumX = 0, sumY = 0, sumZ = 0;
        int distFromBounds = std::min(j, (int)roughPath.poses.size() - j - 1);
        int dynWindow = std::min(windowSize, distFromBounds*2+1);

        // Sum positions within the dynamic window
        for (size_t k = j - dynWindow / 2; k <= j + dynWindow / 2; ++k)
        {
            sumX += roughPath.poses[k].pose.position.x;
            sumY += roughPath.poses[k].pose.position.y;
            sumZ += roughPath.poses[k].pose.position.z;
        }

        // Compute the average and assign it to the smoothed path
        smoothPath.poses[j].pose.position.x = sumX / dynWindow;
        smoothPath.poses[j].pose.position.y = sumY / dynWindow;
        smoothPath.poses[j].pose.position.z = sumZ / dynWindow;
    }

    return smoothPath;
}

nav_msgs::msg::Path removePathOverlap(const nav_msgs::msg::Path &loopPath)
{
    nav_msgs::msg::Path cutLoopPath(loopPath);
    for (auto last = cutLoopPath.poses.end() - 1; last != cutLoopPath.poses.begin(); --last)
    {
        auto [lin_dist1, ang_dist1] = diffBetweenPoses(last->pose, cutLoopPath.poses.begin()->pose);
        auto [lin_dist2, ang_dist2] = diffBetweenPoses(last->pose, (cutLoopPath.poses.begin() + 1)->pose);
        if (lin_dist1 > lin_dist2)
        {
            cutLoopPath.poses.erase(last);
        }
        else
        {
            break;
        }
    }
    return cutLoopPath;
}

#endif // UTILS_HPP