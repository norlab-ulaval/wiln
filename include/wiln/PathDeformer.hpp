#pragma once

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>

namespace wiln {

/**
 * @brief Pure math engine for trajectory deformation and kinematic smoothing.
 * Implements an "Elastic Band" approach with potential fields.
 */
class PathDeformer {
public:
    struct Params {
        double kappa_max = 0.7;        // Max curvature (1/R) for articulated MTT
        double attraction_gain = 1.5;  // Force pulling back to original path
        double repulsion_gain = 2.0;   // Force pushing away from obstacles
        double repulsion_dist = 1.5;   // Distance at which obstacles start repelling
        double internal_force = 0.5;   // Force keeping the band smooth (spring)
    };

    explicit PathDeformer(const Params& params);

    /**
     * @brief Deform a local path based on obstacles.
     * @param original_path The reference path from the teach phase.
     * @param obstacles Point cloud of obstacles (already filtered).
     * @return A smoothed, collision-free path compliant with kappa_max.
     */
    nav_msgs::msg::Path deform(const nav_msgs::msg::Path& original_path,
                               const std::vector<Eigen::Vector3d>& obstacles);

private:
    Params params_;

    // Helper to clamp curvature
    void applyKinematicConstraints(std::vector<Eigen::Vector3d>& points);
    
    // Core elastic band iteration
    void performElasticIteration(std::vector<Eigen::Vector3d>& points,
                                 const std::vector<Eigen::Vector3d>& reference,
                                 const std::vector<Eigen::Vector3d>& obstacles);
};

} // namespace wiln
