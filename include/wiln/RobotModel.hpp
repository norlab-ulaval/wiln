#pragma once
/**
 * RobotModel.hpp — Pluggable robot model for WILN.
 *
 * Provides kinematic limits and footprint geometry used by PathDeformer
 * and the Foxglove visualizer.  All implementations must be thread-safe
 * because the model is updated from the articulation-angle callback and
 * read from the 10 Hz stream loop.
 *
 * Two built-in models:
 *   GenericModel  — curvature-limited single-body ground robot.
 *   MttModel      — articulated tracked vehicle (tractor + trailer).
 */

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace wiln {

// ---------------------------------------------------------------------------
// Abstract base
// ---------------------------------------------------------------------------
class RobotModel {
public:
    virtual ~RobotModel() = default;

    /** Maximum path curvature [1/m] given the current robot state. */
    virtual double kappaMax() const = 0;

    /** Total width of the widest body [m]. */
    virtual double bodyWidth() const = 0;

    /** Name used in diagnostics / log messages. */
    virtual std::string name() const = 0;

    /**
     * Swept-area polygon in the robot (tractor) body frame, XY plane.
     * Vertices in CCW order.  Used by the visualizer; optional for collision.
     */
    virtual std::vector<Eigen::Vector2d> sweptArea() const = 0;
};

// ---------------------------------------------------------------------------
// Generic single-body model
// ---------------------------------------------------------------------------
class GenericModel : public RobotModel {
public:
    struct Params {
        double kappa_max    = 0.7;   // [1/m]
        double length_front = 0.5;   // [m] from centre to front
        double length_rear  = 0.5;   // [m] from centre to rear
        double width        = 1.0;   // [m]
    };

    GenericModel() = default;
    explicit GenericModel(const Params& p) : p_(p) {}

    double kappaMax()   const override { return p_.kappa_max; }
    double bodyWidth()  const override { return p_.width; }
    std::string name()  const override { return "generic"; }

    std::vector<Eigen::Vector2d> sweptArea() const override {
        double h = p_.width / 2.0;
        return {
            { p_.length_front,  h},
            { p_.length_front, -h},
            {-p_.length_rear,  -h},
            {-p_.length_rear,   h},
        };
    }

private:
    Params p_;
};

// ---------------------------------------------------------------------------
// MTT articulated tracked vehicle model
// ---------------------------------------------------------------------------
class MttModel : public RobotModel {
public:
    struct Params {
        // ---- Tractor body ----
        double tractor_length_front = 1.8;   // [m] from rear axle to front
        double tractor_length_rear  = 1.0;   // [m] from rear axle to hitch
        double tractor_width        = 2.5;   // [m]

        // ---- Trailer body ----
        double trailer_length_front = 0.5;   // [m] from hitch point to trailer front
        double trailer_length_rear  = 3.0;   // [m] from hitch point to trailer rear
        double trailer_width        = 2.5;   // [m]

        // ---- Kinematics ----
        double kappa_max_nominal  = 0.70;    // [1/m] at zero articulation
        // Physical articulation limit.  MUST match the path follower's psi_max_rad.
        // Default 0.785 rad (45°) is a generic conservative value. MTT's physical
        // stop is confirmed 45° (2026-07-20); its operational safety limit is
        // smaller still (0.733 rad / 42°) — override via articulation_limit_rad.
        double articulation_limit = 0.785;   // [rad] — override via articulation_limit_rad param
        // Angle above which the curvature budget starts decreasing linearly.
        // Maintained proportionally to the limit (≈ 44 % of articulation_limit).
        double onset_angle        = 0.349;   // [rad] — auto-scaled when articulation_limit is set
    };

    MttModel() : articulation_angle_(0.0) {}
    explicit MttModel(const Params& p) : p_(p), articulation_angle_(0.0) {}

    /** Call from the articulation-angle callback.  Thread-safe (atomic). */
    void updateArticulationAngle(double angle_rad) {
        double clamped = std::max(-p_.articulation_limit,
                                  std::min(p_.articulation_limit, angle_rad));
        articulation_angle_.store(clamped);
    }

    double kappaMax() const override {
        double angle = std::abs(articulation_angle_.load());
        if (angle > p_.onset_angle) {
            double fraction = 1.0 - (angle - p_.onset_angle) /
                              (p_.articulation_limit - p_.onset_angle);
            return p_.kappa_max_nominal * std::max(0.1, fraction);
        }
        return p_.kappa_max_nominal;
    }

    double bodyWidth() const override {
        return std::max(p_.tractor_width, p_.trailer_width);
    }

    std::string name() const override { return "mtt"; }

    std::vector<Eigen::Vector2d> sweptArea() const override {
        double psi = articulation_angle_.load();
        double ca  = std::cos(psi);
        double sa  = std::sin(psi);
        double hw  = p_.tractor_width / 2.0;
        double thw = p_.trailer_width / 2.0;

        // Tractor rectangle (in tractor frame)
        std::vector<Eigen::Vector2d> verts = {
            { p_.tractor_length_front,  hw},
            { p_.tractor_length_front, -hw},
            {-p_.tractor_length_rear,  -hw},
            {-p_.tractor_length_rear,   hw},
        };

        // Trailer rectangle (in trailer frame, then rotate by -psi into tractor frame)
        // Hitch point in tractor frame: x = -tractor_length_rear
        double hitch_x = -p_.tractor_length_rear;
        std::array<Eigen::Vector2d, 4> trailer_local = {{
            { p_.trailer_length_front,  thw},
            { p_.trailer_length_front, -thw},
            {-p_.trailer_length_rear,  -thw},
            {-p_.trailer_length_rear,   thw},
        }};
        for (auto& v : trailer_local) {
            // Rotate by -psi (trailer frame → tractor frame)
            double rx = ca * v.x() + sa * v.y();
            double ry = -sa * v.x() + ca * v.y();
            verts.emplace_back(hitch_x + rx, ry);
        }
        return verts;
    }

private:
    Params p_;
    std::atomic<double> articulation_angle_;
};

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
/**
 * @param type                   "mtt" or "generic"
 * @param kappa_max_override     If > 0, overrides the nominal curvature limit [1/m].
 * @param articulation_limit_rad If > 0 (MttModel only), overrides the physical
 *                               articulation limit [rad].  MUST match the path
 *                               follower's psi_max_rad.  onset_angle is scaled
 *                               proportionally (44 % of the limit).
 */
inline std::shared_ptr<RobotModel> makeRobotModel(
    const std::string& type,
    double kappa_max_override      = 0.0,
    double articulation_limit_rad  = 0.0)
{
    if (type == "mtt") {
        MttModel::Params p;
        if (kappa_max_override     > 0.0) p.kappa_max_nominal  = kappa_max_override;
        if (articulation_limit_rad > 0.0) {
            p.articulation_limit = articulation_limit_rad;
            // Scale onset proportionally: keep the same fraction as the default (≈ 44 %)
            constexpr double kOnsetFraction = 0.349 / 0.785;
            p.onset_angle = kOnsetFraction * articulation_limit_rad;
        }
        return std::make_shared<MttModel>(p);
    }
    GenericModel::Params p;
    if (kappa_max_override > 0.0) p.kappa_max = kappa_max_override;
    return std::make_shared<GenericModel>(p);
}

} // namespace wiln
