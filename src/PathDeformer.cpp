#include "wiln/PathDeformer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wiln {

PathDeformer::PathDeformer(const Params& params) : params_(params) {
    points_buf_.reserve(256);
    reference_buf_.reserve(256);
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
    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed_ms = [&t0]() {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
    };

    // --- Validity guard ---
    if (!isValidPath(original_path)) {
        if (diag) { diag->used_fallback = true; }
        return original_path;
    }

    const size_t N = original_path.poses.size();

    // Populate reusable buffers and reference-path normals.
    points_buf_.resize(N);
    reference_buf_.resize(N);
    std::vector<Eigen::Vector2d> normals(N, Eigen::Vector2d::Zero());
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector3d pt(
            original_path.poses[i].pose.position.x,
            original_path.poses[i].pose.position.y,
            original_path.poses[i].pose.position.z);
        points_buf_[i]    = pt;
        reference_buf_[i] = pt;

        const size_t before = i == 0 ? 0 : i - 1;
        const size_t after = i + 1 < N ? i + 1 : N - 1;
        Eigen::Vector2d tangent(
            original_path.poses[after].pose.position.x -
                original_path.poses[before].pose.position.x,
            original_path.poses[after].pose.position.y -
                original_path.poses[before].pose.position.y);
        if (tangent.norm() > params_.min_segment_length) {
            tangent.normalize();
            normals[i] = Eigen::Vector2d(-tangent.y(), tangent.x());
        }
    }

    if (diag) {
        diag->obstacle_count  = obstacles.size();
        diag->horizon_points  = N;
    }

    if (obstacles.empty()) {
        if (diag) {
            diag->deform_time_ms = elapsed_ms();
            diag->path_is_clear = true;
        }
        return original_path;
    }

    // Build density-invariant lateral requirements. Summing every LiDAR point
    // made the old force saturate according to cloud density and flip side at
    // adjacent samples. Here the whole horizon chooses one bypass side.
    const double robot_half_width = robot_model_ ? 0.5 * robot_model_->bodyWidth() : 0.5;
    const double required_clearance = robot_half_width + params_.obstacle_margin;
    std::vector<double> left_need(N, 0.0);
    std::vector<double> right_need(N, 0.0);
    bool obstacle_relevant = false;

    for (size_t i = 1; i + 1 < N; ++i) {
        if (normals[i].squaredNorm() < 0.5) continue;
        const Eigen::Vector2d tangent(normals[i].y(), -normals[i].x());
        const Eigen::Vector2d path_xy(reference_buf_[i].x(), reference_buf_[i].y());
        for (const auto& obstacle : obstacles) {
            const Eigen::Vector2d delta = obstacle.head<2>() - path_xy;
            const double longitudinal = delta.dot(tangent);
            const double lateral = delta.dot(normals[i]);
            if (std::abs(longitudinal) > params_.influence_longitudinal ||
                std::abs(lateral) > params_.repulsion_dist)
            {
                continue;
            }
            obstacle_relevant = true;
            const double phase = M_PI * std::abs(longitudinal) /
                std::max(params_.influence_longitudinal, 1e-6);
            const double profile = 0.5 * (1.0 + std::cos(phase));
            left_need[i] = std::max(
                left_need[i], profile * (lateral + required_clearance));
            right_need[i] = std::max(
                right_need[i], profile * (required_clearance - lateral));
        }
    }

    if (!obstacle_relevant) {
        if (diag) {
            diag->deform_time_ms = elapsed_ms();
            diag->path_is_clear = true;
        }
        return original_path;
    }

    const double left_peak = *std::max_element(left_need.begin(), left_need.end());
    const double right_peak = *std::max_element(right_need.begin(), right_need.end());
    const int side = left_peak < right_peak ? 1 : -1;
    const auto& selected_need = side > 0 ? left_need : right_need;
    const double selected_peak = side > 0 ? left_peak : right_peak;
    const double clearance_gain = std::max(params_.repulsion_gain, 1.0);
    if (diag) diag->avoidance_side = side;

    // If neither side fits inside the configured corridor, refuse to invent a
    // clipped path. The independent obstacle stop remains authoritative.
    if (selected_peak * clearance_gain > params_.max_total_deformation + 1e-6) {
        if (diag) {
            diag->used_fallback = true;
            diag->path_is_clear = false;
            diag->deform_time_ms = elapsed_ms();
        }
        return original_path;
    }

    std::vector<double> target_offset(N, 0.0);
    std::vector<double> offset(N, 0.0);
    for (size_t i = 1; i + 1 < N; ++i) {
        target_offset[i] = static_cast<double>(side) *
            std::clamp(
            selected_need[i] * clearance_gain,
            0.0,
            params_.max_total_deformation);
        offset[i] = target_offset[i];
    }

    // Solve a one-dimensional elastic band over lateral offsets. A single
    // signed field plus neighbour regularization cannot create alternating
    // left/right spikes. Anchors remain exactly on the Teach path.
    for (int iteration = 0; iteration < params_.max_iterations; ++iteration) {
        if ((iteration & 3) == 0 && elapsed_ms() > params_.time_budget_ms) {
            if (diag) {
                diag->time_budget_exceeded = true;
                diag->used_fallback = true;
                diag->path_is_clear = false;
                diag->deform_time_ms = elapsed_ms();
            }
            return original_path;
        }
        std::vector<double> next = offset;
        for (size_t i = 1; i + 1 < N; ++i) {
            const double denominator = params_.attraction_gain +
                2.0 * params_.internal_force;
            const double equilibrium =
                (params_.attraction_gain * target_offset[i] +
                params_.internal_force * (offset[i - 1] + offset[i + 1])) /
                std::max(denominator, 1e-6);
            const double step = std::clamp(
                equilibrium - offset[i],
                -params_.max_deformation_step,
                params_.max_deformation_step);
            next[i] = std::clamp(
                offset[i] + params_.step_size * step,
                -params_.max_total_deformation,
                params_.max_total_deformation);
        }
        offset.swap(next);
    }

    double max_disp = 0.0;
    for (size_t i = 1; i + 1 < N; ++i) {
        points_buf_[i].x() += normals[i].x() * offset[i];
        points_buf_[i].y() += normals[i].y() * offset[i];
        max_disp = std::max(max_disp, std::abs(offset[i]));
    }

    double max_curvature = 0.0;
    for (size_t i = 1; i + 1 < N; ++i) {
        const Eigen::Vector2d a = points_buf_[i].head<2>() - points_buf_[i - 1].head<2>();
        const Eigen::Vector2d b = points_buf_[i + 1].head<2>() - points_buf_[i].head<2>();
        const Eigen::Vector2d c = points_buf_[i + 1].head<2>() - points_buf_[i - 1].head<2>();
        const double denominator = a.norm() * b.norm() * c.norm();
        if (denominator <= 1e-9) continue;
        const double cross = a.x() * b.y() - a.y() * b.x();
        max_curvature = std::max(max_curvature, std::abs(2.0 * cross / denominator));
    }

    double min_clearance = std::numeric_limits<double>::infinity();
    for (const auto& point : points_buf_) {
        for (const auto& obstacle : obstacles) {
            min_clearance = std::min(
                min_clearance, (point - obstacle).head<2>().norm());
        }
    }
    const bool curvature_feasible = max_curvature <= effectiveKappaMax() * 1.05;
    const bool clearance_feasible = min_clearance >= required_clearance;
    if (diag) {
        diag->max_displacement_m = max_disp;
        diag->min_clearance_m = min_clearance;
        diag->max_curvature_m_inv = max_curvature;
        diag->path_is_clear = curvature_feasible && clearance_feasible;
    }
    if (!curvature_feasible || !clearance_feasible) {
        if (diag) {
            diag->used_fallback = true;
            diag->deform_time_ms = elapsed_ms();
        }
        return original_path;
    }

    nav_msgs::msg::Path result = original_path;
    for (size_t i = 0; i < N; ++i) {
        result.poses[i].pose.position.x = points_buf_[i].x();
        result.poses[i].pose.position.y = points_buf_[i].y();
        // Z preserved from original (terrain following) — XY-only deformation
        result.poses[i].pose.position.z = reference_buf_[i].z();
    }

    updateOrientations(result, points_buf_);

    if (diag) {
        diag->deform_time_ms = elapsed_ms();
    }

    return result;
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
