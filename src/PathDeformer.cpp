#include "wiln/PathDeformer.hpp"
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wiln {

PathDeformer::PathDeformer(const Params& params) : params_(params) {
    points_buf_.reserve(256);
    reference_buf_.reserve(256);
    new_points_buf_.reserve(256);
}

// ---------------------------------------------------------------------------
double PathDeformer::effectiveKappaMax() const {
    if (robot_model_) return robot_model_->kappaMax();
    return 0.7;  // sensible default
}

// ---------------------------------------------------------------------------
bool PathDeformer::isValidPath(const nav_msgs::msg::Path& path) {
    if (path.poses.size() < 3) return false;
    for (const auto& ps : path.poses) {
        const auto& p = ps.pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
nav_msgs::msg::Path PathDeformer::deform(const nav_msgs::msg::Path&         original_path,
                                          const std::vector<Eigen::Vector3d>& obstacles,
                                          Diag* diag)
{
    auto t0 = std::chrono::steady_clock::now();

    // --- Validity guard ---
    if (!isValidPath(original_path)) {
        if (diag) { diag->used_fallback = true; }
        return original_path;
    }

    const size_t N = original_path.poses.size();

    // --- Populate buffers (reuse existing capacity) ---
    points_buf_.resize(N);
    reference_buf_.resize(N);
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector3d pt(
            original_path.poses[i].pose.position.x,
            original_path.poses[i].pose.position.y,
            original_path.poses[i].pose.position.z);
        points_buf_[i]    = pt;
        reference_buf_[i] = pt;
    }

    if (diag) {
        diag->obstacle_count  = obstacles.size();
        diag->horizon_points  = N;
    }

    // --- Elastic band iterations ---
    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        // Time-budget check (every 2 iterations to amortize chrono overhead)
        if ((iter & 1) == 0) {
            auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed > params_.time_budget_ms) {
                if (diag) { diag->time_budget_exceeded = true; diag->used_fallback = true; }
                return original_path;
            }
        }

        performElasticIteration(points_buf_, reference_buf_, obstacles);
        applyKinematicConstraints(points_buf_);
    }

    // --- Max total deformation guard ---
    double max_disp = 0.0;
    for (size_t i = 1; i < N - 1; ++i) {  // skip anchors
        double d = (points_buf_[i] - reference_buf_[i]).head<2>().norm();  // XY only
        max_disp = std::max(max_disp, d);
        if (d > params_.max_total_deformation) {
            // Reset this point back to reference
            points_buf_[i] = reference_buf_[i];
        }
    }

    // --- Build result ---
    nav_msgs::msg::Path result = original_path;
    for (size_t i = 0; i < N; ++i) {
        result.poses[i].pose.position.x = points_buf_[i].x();
        result.poses[i].pose.position.y = points_buf_[i].y();
        // Z preserved from original (terrain following) — XY-only deformation
        result.poses[i].pose.position.z = reference_buf_[i].z();
    }

    updateOrientations(result, points_buf_);

    auto t1 = std::chrono::steady_clock::now();
    if (diag) {
        diag->deform_time_ms     = std::chrono::duration<double, std::milli>(t1 - t0).count();
        diag->max_displacement_m = max_disp;
    }

    return result;
}

// ---------------------------------------------------------------------------
void PathDeformer::performElasticIteration(std::vector<Eigen::Vector3d>&       points,
                                            const std::vector<Eigen::Vector3d>& reference,
                                            const std::vector<Eigen::Vector3d>& obstacles)
{
    const size_t N = points.size();
    new_points_buf_.resize(N);
    new_points_buf_ = points;

    // Skip hard anchors at index 0 and N-1
    for (size_t i = 1; i < N - 1; ++i) {
        Eigen::Vector3d force = Eigen::Vector3d::Zero();

        // 1. Internal (spring smoothing) — 3D
        force += params_.internal_force * (points[i-1] + points[i+1] - 2.0 * points[i]);

        // 2. Attraction back to reference — 3D (keeps Z as well)
        force += params_.attraction_gain * (reference[i] - points[i]);

        // 3. Repulsion from obstacles — XY only (no Z component)
        for (const auto& obs : obstacles) {
            Eigen::Vector2d diff2d(points[i].x() - obs.x(),
                                   points[i].y() - obs.y());
            double dist2d = diff2d.norm();
            if (dist2d < params_.repulsion_dist && dist2d > 1e-4) {
                double magnitude = params_.repulsion_gain *
                                   (params_.repulsion_dist - dist2d) / dist2d;
                // Apply only in XY
                force.x() += magnitude * diff2d.x();
                force.y() += magnitude * diff2d.y();
                // force.z() intentionally untouched
            }
        }

        // Clamp per-step displacement
        Eigen::Vector3d delta = params_.step_size * force;
        double delta_norm = delta.norm();
        if (delta_norm > params_.max_deformation_step) {
            delta *= params_.max_deformation_step / delta_norm;
        }

        new_points_buf_[i] = points[i] + delta;
    }
    points = new_points_buf_;
}

// ---------------------------------------------------------------------------
void PathDeformer::applyKinematicConstraints(std::vector<Eigen::Vector3d>& points)
{
    const size_t N = points.size();
    double kappa_max = effectiveKappaMax();

    for (size_t i = 1; i < N - 1; ++i) {
        Eigen::Vector3d v1 = points[i]   - points[i-1];
        Eigen::Vector3d v2 = points[i+1] - points[i];

        double d1 = v1.norm();
        if (d1 < params_.min_segment_length) continue;

        // Numerically stable angle via atan2 — never NaN, stable near 0 and π
        double cross_z = v1.x() * v2.y() - v1.y() * v2.x();  // 2D cross product z
        double dot_xy  = v1.x() * v2.x() + v1.y() * v2.y();
        double angle = std::atan2(std::abs(cross_z), dot_xy);
        double max_angle = kappa_max * d1;

        if (angle > max_angle && std::isfinite(max_angle)) {
            double yaw1 = std::atan2(v1.y(), v1.x());
            double yaw2 = std::atan2(v2.y(), v2.x());
            double dyaw = yaw2 - yaw1;
            while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

            double clamped_yaw = yaw1 + (dyaw >= 0.0 ? max_angle : -max_angle);
            double len2 = v2.norm();
            if (len2 > params_.min_segment_length) {
                points[i+1].x() = points[i].x() + len2 * std::cos(clamped_yaw);
                points[i+1].y() = points[i].y() + len2 * std::sin(clamped_yaw);
                // Z of i+1 preserved
            }
        }
    }
}

// ---------------------------------------------------------------------------
void PathDeformer::updateOrientations(nav_msgs::msg::Path&                path,
                                       const std::vector<Eigen::Vector3d>& points)
{
    const size_t N = points.size();
    const Eigen::Vector3d world_up(0.0, 0.0, 1.0);

    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector3d tangent;
        if (i == 0) {
            tangent = points[1] - points[0];
        } else if (i == N - 1) {
            tangent = points[N-1] - points[N-2];
        } else {
            tangent = points[i+1] - points[i-1];
        }

        double len = tangent.norm();
        if (len < 1e-6) continue;
        tangent /= len;

        // Frenet-Serret: right = tangent × up, then recompute up
        Eigen::Vector3d right = tangent.cross(world_up);
        double right_len = right.norm();
        if (right_len < 1e-6) {
            // Near-vertical tangent: fall back to yaw-only
            tf2::Quaternion q;
            q.setRPY(0, 0, std::atan2(tangent.y(), tangent.x()));
            path.poses[i].pose.orientation = tf2::toMsg(q);
            continue;
        }
        right /= right_len;
        Eigen::Vector3d up = right.cross(tangent);

        // ROS convention: x-forward, y-left, z-up
        Eigen::Matrix3d R;
        R.col(0) =  tangent;
        R.col(1) = -right;
        R.col(2) =  up;

        Eigen::Quaterniond q(R);
        q.normalize();
        // Validate quaternion before writing
        if (!std::isfinite(q.x()) || !std::isfinite(q.y()) ||
            !std::isfinite(q.z()) || !std::isfinite(q.w())) continue;

        path.poses[i].pose.orientation.x = q.x();
        path.poses[i].pose.orientation.y = q.y();
        path.poses[i].pose.orientation.z = q.z();
        path.poses[i].pose.orientation.w = q.w();
    }
}

} // namespace wiln
