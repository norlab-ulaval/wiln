#pragma once
/**
 * PathDeformer.hpp — Elastic-band path deformation.
 *
 * Key correctness properties enforced here:
 *   - First and last path points are hard anchors (never moved).
 *   - Obstacle repulsion is applied in XY only; Z is preserved from the
 *     original path (terrain following).
 *   - Force step size and total displacement are clamped.
 *   - Output orientations use a Frenet-Serret adapted frame (3D-correct).
 *   - A time budget guards the 10 Hz loop; if exceeded the original horizon
 *     is returned unchanged and the caller is informed.
 *
 * The replay node publishes the result on /wiln/control/local_plan. When the
 * path follower is configured with that topic, it uses the fresh deformed
 * horizon for steering and holds zero if the stream later becomes stale.
 */

#include "wiln/RobotModel.hpp"
#include <Eigen/Dense>
#include <limits>
#include <nav_msgs/msg/path.hpp>
#include <vector>

namespace wiln {

class PathDeformer {
public:
    struct Params {
        // ---- Force gains ----
        double attraction_gain  = 1.5;   // pull back to reference path
        double repulsion_gain   = 2.0;   // push away from obstacles
        double repulsion_dist   = 1.5;   // [m] repulsion onset distance
        double internal_force   = 0.5;   // spring smoothing
        double influence_longitudinal = 3.0; // [m] smooth bypass lead/trail distance
        double obstacle_margin        = 0.25; // [m] outside robot half-width

        // ---- Iteration control ----
        int    max_iterations          = 5;
        double step_size               = 0.1;    // per-iteration step scale
        double max_deformation_step    = 0.25;   // [m] max displacement per iteration
        double max_total_deformation   = 1.0;    // [m] max displacement from original

        // ---- Safety ----
        double time_budget_ms          = 8.0;    // [ms] fallback if exceeded
        double min_segment_length      = 0.01;   // [m] reject degenerate segments
    };

    // ---- Diagnostic output ----
    struct Diag {
        double deform_time_ms       = 0.0;
        size_t obstacle_count       = 0;
        size_t horizon_points       = 0;
        double max_displacement_m   = 0.0;
        double min_clearance_m      = std::numeric_limits<double>::infinity();
        double max_curvature_m_inv  = 0.0;
        int    avoidance_side       = 0;  // +1 left, -1 right, 0 no bypass
        bool   path_is_clear        = true;
        bool   time_budget_exceeded = false;
        bool   used_fallback        = false;
    };

    explicit PathDeformer(const Params& params);

    /** Inject robot model (owned by WilnNode — lifetime guaranteed). */
    void setRobotModel(RobotModel* model) { robot_model_ = model; }

    /**
     * Deform the local horizon around obstacles.
     * @param original_path  Reference path (from TeachRecorder / TrajectoryStreamer).
     * @param obstacles      Pre-filtered obstacles in the same frame as the path.
     * @param diag           Optional diagnostics (Foxglove).
     * @return Deformed path.  On fallback, returns original_path unchanged.
     */
    nav_msgs::msg::Path deform(const nav_msgs::msg::Path&         original_path,
                               const std::vector<Eigen::Vector3d>& obstacles,
                               Diag* diag = nullptr);

private:
    Params       params_;
    RobotModel*  robot_model_ = nullptr;   // non-owning, may be null

    // Pre-allocated buffers (avoid heap inside 10 Hz loop)
    std::vector<Eigen::Vector3d> points_buf_;
    std::vector<Eigen::Vector3d> reference_buf_;

    double effectiveKappaMax() const;

    // Frenet-Serret adapted frames for correct 3D orientation
    void updateOrientations(nav_msgs::msg::Path&                     path,
                            const std::vector<Eigen::Vector3d>&      points);

    // Input validity
    static bool isValidPath(const nav_msgs::msg::Path& path);
};

} // namespace wiln
