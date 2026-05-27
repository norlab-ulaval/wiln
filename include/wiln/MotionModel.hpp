#pragma once
/**
 * MotionModel.hpp -- Header-only kinematic model utilities.
 *
 * Port of mtt_bringup/motion_model.py.
 * All functions are inline and depend only on standard math.
 */

#include <algorithm>
#include <cmath>

namespace wiln {

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
struct MotionModelParams {
    double wheelbase_m              = 2.4;
    double max_articulation_rad     = 1.047;  // 60 degrees (MTT)
    double min_turn_speed_ms        = 0.25;
    bool   use_slip_heuristic       = true;
    double yaw_slip_base            = 0.10;
    double yaw_slip_speed_gain      = 0.05;
    double yaw_slip_articulation_gain = 0.15;
    double yaw_slip_min_scale       = 0.55;
};

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------
inline double mmClamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double wrapToPi(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ---------------------------------------------------------------------------
// Articulation <-> curvature conversion
// ---------------------------------------------------------------------------

/** Compute articulation angle from desired path curvature. */
inline double articulationFromCurvature(double kappa, const MotionModelParams& p) {
    if (std::abs(kappa) < 1e-12 || p.wheelbase_m <= 1e-9) return 0.0;
    return mmClamp(std::atan(kappa * p.wheelbase_m),
                   -p.max_articulation_rad, p.max_articulation_rad);
}

/** Normalize articulation angle to [-1, 1] steering command. */
inline double normalizedSteerFromArticulation(double psi_rad, const MotionModelParams& p) {
    return mmClamp(psi_rad / std::max(p.max_articulation_rad, 1e-6), -1.0, 1.0);
}

// ---------------------------------------------------------------------------
// Slip compensation
// ---------------------------------------------------------------------------

/**
 * Returns a scale factor < 1.0 modeling yaw slip on the articulated vehicle.
 * When slip > 1, the robot turns less than commanded; we compensate by
 * commanding more curvature.  Scale is applied as: kappa_cmd = kappa / slip.
 *
 * Port of motion_model.py::slip_scale().
 */
inline double slipScale(double speed_ms, double psi_rad, const MotionModelParams& p) {
    if (!p.use_slip_heuristic) return 1.0;
    double norm_psi = std::abs(psi_rad) / std::max(p.max_articulation_rad, 1e-6);
    double scale = 1.0
        - p.yaw_slip_base
        - p.yaw_slip_speed_gain * std::abs(speed_ms)
        - p.yaw_slip_articulation_gain * norm_psi;
    return mmClamp(scale, p.yaw_slip_min_scale, 1.0);
}

} // namespace wiln
