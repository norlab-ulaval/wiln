#pragma once

/**
 * SE3Utils.hpp — Pure Lie algebra utilities for SE(3) / SO(3).
 *
 * No ROS dependency — only Eigen.
 *
 * Conventions:
 *   T ∈ SE(3) stored as 4×4 homogeneous matrix
 *   xi ∈ se(3)  stored as 6-vector [ρ₁,ρ₂,ρ₃, ω₁,ω₂,ω₃]
 *               where ω is the rotation axis-angle, ρ is the translational part
 *               (left-perturbation convention used throughout)
 *
 * References:
 *   Chirikjian, G. — Stochastic Models, Information Theory, and Lie Groups
 *   Barfoot, T.  — State Estimation for Robotics (Cambridge, 2017)
 */

#include <Eigen/Dense>
#include <cmath>

namespace wiln {
namespace se3 {

// ---------------------------------------------------------------------------
// SO(3) utilities
// ---------------------------------------------------------------------------

/** Skew-symmetric matrix of a 3-vector (hat operator for so(3)). */
inline Eigen::Matrix3d hat3(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<    0, -v(2),  v(1),
         v(2),     0, -v(0),
        -v(1),  v(0),     0;
    return S;
}

/** Inverse hat: extract axis-angle vector from skew-symmetric matrix. */
inline Eigen::Vector3d vee3(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(S(2,1), S(0,2), S(1,0));
}

/**
 * Rodrigues rotation formula: Exp map so(3) → SO(3).
 * @param omega  Rotation vector (axis × angle) in R³.
 */
inline Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-9) {
        return Eigen::Matrix3d::Identity() + hat3(omega);
    }
    Eigen::Vector3d axis = omega / theta;
    double c = std::cos(theta);
    double s = std::sin(theta);
    return c * Eigen::Matrix3d::Identity()
         + s * hat3(axis)
         + (1.0 - c) * (axis * axis.transpose());
}

/**
 * Log map SO(3) → so(3).
 * @return axis-angle vector; norm = rotation angle in [0, π].
 */
inline Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R) {
    double cos_theta = 0.5 * (R.trace() - 1.0);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));  // clamp for safety
    double theta = std::acos(cos_theta);
    if (theta < 1e-9) {
        return vee3(0.5 * (R - R.transpose()));  // first-order approximation
    }
    if (std::abs(theta - M_PI) < 1e-6) {
        // Degenerate case: 180° rotation — recover axis from diagonal
        int k = 0;
        if (R(1,1) > R(0,0)) k = 1;
        if (R(2,2) > R(k,k)) k = 2;
        Eigen::Vector3d axis;
        axis(k) = std::sqrt((R(k,k) - cos_theta) / (1.0 - cos_theta));
        for (int j = 0; j < 3; ++j) {
            if (j != k) axis(j) = R(j,k) / (2.0 * axis(k) * (1.0 - cos_theta));
        }
        return M_PI * axis.normalized();
    }
    return (theta / (2.0 * std::sin(theta))) * vee3(R - R.transpose());
}

/**
 * Left Jacobian of SO(3).
 * Maps from se(3) perturbation to SE(3) Lie bracket.
 */
inline Eigen::Matrix3d LeftJacobianSO3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-9) {
        return Eigen::Matrix3d::Identity() + 0.5 * hat3(omega);
    }
    Eigen::Vector3d axis = omega / theta;
    double s = std::sin(theta);
    double c = std::cos(theta);
    return (s / theta) * Eigen::Matrix3d::Identity()
         + (1.0 - s / theta) * (axis * axis.transpose())
         + ((1.0 - c) / theta) * hat3(axis);
}

/** Inverse of the left Jacobian of SO(3). */
inline Eigen::Matrix3d LeftJacobianInvSO3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-9) {
        return Eigen::Matrix3d::Identity() - 0.5 * hat3(omega);
    }
    Eigen::Vector3d axis = omega / theta;
    double half_theta = 0.5 * theta;
    return (half_theta * std::cos(half_theta) / std::sin(half_theta)) * Eigen::Matrix3d::Identity()
         + (1.0 - half_theta * std::cos(half_theta) / std::sin(half_theta)) * (axis * axis.transpose())
         - half_theta * hat3(axis);
}

// ---------------------------------------------------------------------------
// SE(3) utilities
// ---------------------------------------------------------------------------

/**
 * Exp map se(3) → SE(3).
 * @param xi  6-vector [ρ; ω] where ω is rotation axis-angle, ρ is translational component.
 * @return 4×4 homogeneous matrix T ∈ SE(3).
 */
inline Eigen::Matrix4d ExpSE3(const Eigen::Vector<double,6>& xi) {
    Eigen::Vector3d rho   = xi.head<3>();
    Eigen::Vector3d omega = xi.tail<3>();
    Eigen::Matrix3d R     = ExpSO3(omega);
    Eigen::Vector3d t     = LeftJacobianSO3(omega) * rho;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>()  = R;
    T.topRightCorner<3,1>() = t;
    return T;
}

/**
 * Log map SE(3) → se(3).
 * @return 6-vector [ρ; ω].
 */
inline Eigen::Vector<double,6> LogSE3(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.topLeftCorner<3,3>();
    Eigen::Vector3d t = T.topRightCorner<3,1>();

    Eigen::Vector3d omega = LogSO3(R);
    Eigen::Vector3d rho   = LeftJacobianInvSO3(omega) * t;

    Eigen::Vector<double,6> xi;
    xi.head<3>() = rho;
    xi.tail<3>() = omega;
    return xi;
}

/** SE(3) inverse. */
inline Eigen::Matrix4d InvSE3(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d Rt = T.topLeftCorner<3,3>().transpose();
    Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
    inv.topLeftCorner<3,3>()  = Rt;
    inv.topRightCorner<3,1>() = -Rt * T.topRightCorner<3,1>();
    return inv;
}

/**
 * Geodesic interpolation on SE(3) (SLERP-like).
 * @param T1, T2  Start and end transforms.
 * @param t       Parameter in [0, 1].
 */
inline Eigen::Matrix4d InterpolateSE3(
    const Eigen::Matrix4d& T1,
    const Eigen::Matrix4d& T2,
    double t)
{
    return T1 * ExpSE3(t * LogSE3(InvSE3(T1) * T2));
}

// ---------------------------------------------------------------------------
// ROS ↔ SE(3) conversions (header-only, no ROS dependency via template tricks)
// These are intentionally generic — pass geometry_msgs::msg::Pose directly.
// ---------------------------------------------------------------------------

/**
 * Convert a pose (position + quaternion) to a 4×4 SE(3) matrix.
 * PoseT must have fields: position.{x,y,z} and orientation.{x,y,z,w}.
 */
template<typename PoseT>
inline Eigen::Matrix4d fromPose(const PoseT& pose) {
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;
    double qw = pose.orientation.w;

    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>()  = q.toRotationMatrix();
    T.topRightCorner<3,1>() = Eigen::Vector3d(
        pose.position.x, pose.position.y, pose.position.z);
    return T;
}

/**
 * Convert a 4×4 SE(3) matrix back to a pose.
 * PoseT must have mutable fields: position.{x,y,z} and orientation.{x,y,z,w}.
 */
template<typename PoseT>
inline PoseT toPose(const Eigen::Matrix4d& T) {
    PoseT pose;
    Eigen::Quaterniond q(T.topLeftCorner<3,3>());
    q.normalize();
    pose.position.x    = T(0,3);
    pose.position.y    = T(1,3);
    pose.position.z    = T(2,3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

} // namespace se3
} // namespace wiln
