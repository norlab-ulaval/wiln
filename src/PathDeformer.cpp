#include "wiln/PathDeformer.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wiln {

PathDeformer::PathDeformer(const Params& params) : params_(params) {}

nav_msgs::msg::Path PathDeformer::deform(const nav_msgs::msg::Path& original_path,
                                        const std::vector<Eigen::Vector3d>& obstacles) {
    if (original_path.poses.size() < 3) return original_path;

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> reference;
    for (const auto& p : original_path.poses) {
        Eigen::Vector3d pt(p.pose.position.x, p.pose.position.y, p.pose.position.z);
        points.push_back(pt);
        reference.push_back(pt);
    }

    // Perform a few iterations of Elastic Band
    for (int iter = 0; iter < 5; ++iter) {
        performElasticIteration(points, reference, obstacles);
        applyKinematicConstraints(points);
    }

    nav_msgs::msg::Path result = original_path;
    for (size_t i = 0; i < points.size(); ++i) {
        result.poses[i].pose.position.x = points[i].x();
        result.poses[i].pose.position.y = points[i].y();
        result.poses[i].pose.position.z = points[i].z();
        
        // Update orientation based on next point (tangent)
        if (i < points.size() - 1) {
            double yaw = std::atan2(points[i+1].y() - points[i].y(), points[i+1].x() - points[i].x());
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            result.poses[i].pose.orientation = tf2::toMsg(q);
        }
    }
    return result;
}

void PathDeformer::performElasticIteration(std::vector<Eigen::Vector3d>& points,
                                           const std::vector<Eigen::Vector3d>& reference,
                                           const std::vector<Eigen::Vector3d>& obstacles) {
    std::vector<Eigen::Vector3d> new_points = points;
    
    // Skip first and last points (anchors)
    for (size_t i = 1; i < points.size() - 1; ++i) {
        Eigen::Vector3d force(0, 0, 0);

        // 1. Internal Force (Smoothing/Spring)
        force += params_.internal_force * (points[i-1] + points[i+1] - 2.0 * points[i]);

        // 2. Attraction Force (Back to original)
        force += params_.attraction_gain * (reference[i] - points[i]);

        // 3. Repulsion Force (Obstacles)
        for (const auto& obs : obstacles) {
            Eigen::Vector3d diff = points[i] - obs;
            double dist = diff.norm();
            if (dist < params_.repulsion_dist && dist > 0.001) {
                force += params_.repulsion_gain * (params_.repulsion_dist - dist) * (diff / dist);
            }
        }

        new_points[i] += 0.1 * force; // Step size
    }
    points = new_points;
}

void PathDeformer::applyKinematicConstraints(std::vector<Eigen::Vector3d>& points) {
    // Basic curvature clamping for articulated robots
    // We ensure that the angle between segments is limited
    for (size_t i = 1; i < points.size() - 1; ++i) {
        Eigen::Vector3d v1 = points[i] - points[i-1];
        Eigen::Vector3d v2 = points[i+1] - points[i];
        
        double d1 = v1.norm();
        if (d1 < 0.001) continue;

        double max_angle = params_.kappa_max * d1; // Delta theta = kappa * distance
        
        double angle = std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
        if (std::isnan(angle)) continue;

        if (angle > max_angle) {
            // Rotate v2 towards v1 to satisfy the constraint
            double factor = max_angle / angle;
            // This is a simplified 2D rotation for the x-y plane
            double yaw1 = std::atan2(v1.y(), v1.x());
            double yaw2 = std::atan2(v2.y(), v2.x());
            double dyaw = yaw2 - yaw1;
            if (dyaw > M_PI) dyaw -= 2 * M_PI;
            if (dyaw < -M_PI) dyaw += 2 * M_PI;
            
            double new_yaw2 = yaw1 + (dyaw > 0 ? max_angle : -max_angle);
            points[i+1].x() = points[i].x() + v2.norm() * std::cos(new_yaw2);
            points[i+1].y() = points[i].y() + v2.norm() * std::sin(new_yaw2);
        }
    }
}

} // namespace wiln
