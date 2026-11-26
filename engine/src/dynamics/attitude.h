#pragma once

#include "dynamics/state_indices.h"
#include "integrator.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Unit quaternion, scalar first. It rotates a body vector into inertial
// axes with the same ZYX sequence the old Euler matrix used: yaw, then
// pitch, then roll. q and -q are the same rotation.

struct Quat {
    double w;
    double x;
    double y;
    double z;
};

inline Quat quat_identity() {
    Quat q;
    q.w = 1.0;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    return q;
}

inline Quat quat_mul(Quat a, Quat b) {
    Quat out;
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return out;
}

inline Quat quat_normalize(Quat q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) return quat_identity();
    Quat out;
    out.w = q.w / n;
    out.x = q.x / n;
    out.y = q.y / n;
    out.z = q.z / n;
    return out;
}

inline Quat quat_from_state(const State& state) {
    Quat q;
    q.w = state[QW];
    q.x = state[QX];
    q.y = state[QY];
    q.z = state[QZ];
    return q;
}

inline void quat_write(State& state, Quat q) {
    state[QW] = q.w;
    state[QX] = q.x;
    state[QY] = q.y;
    state[QZ] = q.z;
}

// q = q_yaw ⊗ q_pitch ⊗ q_roll. Angles are radians.
inline Quat quat_from_euler(double roll, double pitch, double yaw) {
    const double half_roll = 0.5 * roll;
    const double half_pitch = 0.5 * pitch;
    const double half_yaw = 0.5 * yaw;

    Quat qx;
    qx.w = std::cos(half_roll);
    qx.x = std::sin(half_roll);
    qx.y = 0.0;
    qx.z = 0.0;

    Quat qy;
    qy.w = std::cos(half_pitch);
    qy.x = 0.0;
    qy.y = std::sin(half_pitch);
    qy.z = 0.0;

    Quat qz;
    qz.w = std::cos(half_yaw);
    qz.x = 0.0;
    qz.y = 0.0;
    qz.z = std::sin(half_yaw);

    return quat_mul(qz, quat_mul(qy, qx));
}

// Body vector to inertial. q is assumed normalized by the caller when it
// came from a long integration; this still treats a short q as identity.
inline void rotate_body_to_inertial(
    Quat q,
    double bx,
    double by,
    double bz,
    double& ix,
    double& iy,
    double& iz
) {
    q = quat_normalize(q);
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - w * z);
    const double r02 = 2.0 * (x * z + w * y);
    const double r10 = 2.0 * (x * y + w * z);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - w * x);
    const double r20 = 2.0 * (x * z - w * y);
    const double r21 = 2.0 * (y * z + w * x);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    ix = r00 * bx + r01 * by + r02 * bz;
    iy = r10 * bx + r11 * by + r12 * bz;
    iz = r20 * bx + r21 * by + r22 * bz;
}

// Roll, pitch, yaw in radians. Pitch stays in [-π/2, π/2]. Past that,
// the quaternion is the continuous state and these three angles fold.
inline void euler_from_quat(Quat q, double& roll, double& pitch, double& yaw) {
    q = quat_normalize(q);
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r10 = 2.0 * (x * y + w * z);
    const double r20 = 2.0 * (x * z - w * y);
    const double r21 = 2.0 * (y * z + w * x);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    const double pitch_cos = std::sqrt(r21 * r21 + r22 * r22);
    pitch = std::atan2(-r20, pitch_cos);
    roll = std::atan2(r21, r22);
    yaw = std::atan2(r10, r00);
}

// Inverse of rotate_body_to_inertial. q takes body vectors into inertial axes.
inline void rotate_inertial_to_body(
    Quat q,
    double ix,
    double iy,
    double iz,
    double& bx,
    double& by,
    double& bz
) {
    q = quat_normalize(q);
    Quat inv;
    inv.w = q.w;
    inv.x = -q.x;
    inv.y = -q.y;
    inv.z = -q.z;
    rotate_body_to_inertial(inv, ix, iy, iz, bx, by, bz);
}

// Smallest rotation that lays body +z on the inertial unit vector (ux, uy, uz).
// The twist about that axis stays whatever the body already has.
inline Quat quat_align_body_z(double ux, double uy, double uz, Quat current) {
    current = quat_normalize(current);
    double zx = 0.0;
    double zy = 0.0;
    double zz = 0.0;
    rotate_body_to_inertial(current, 0.0, 0.0, 1.0, zx, zy, zz);

    const double cos_angle = std::clamp(zx * ux + zy * uy + zz * uz, -1.0, 1.0);
    const double ax = zy * uz - zz * uy;
    const double ay = zz * ux - zx * uz;
    const double az = zx * uy - zy * ux;
    const double sin_angle = std::sqrt(ax * ax + ay * ay + az * az);

    if (sin_angle < 1e-12) {
        if (cos_angle > 0.0) return current;
        Quat flip;
        flip.w = 0.0;
        flip.x = 1.0;
        flip.y = 0.0;
        flip.z = 0.0;
        return quat_normalize(quat_mul(current, flip));
    }

    const double angle = std::atan2(sin_angle, cos_angle);
    const double half = 0.5 * angle;
    const double scale = std::sin(half) / sin_angle;
    Quat delta;
    delta.w = std::cos(half);
    delta.x = ax * scale;
    delta.y = ay * scale;
    delta.z = az * scale;
    return quat_normalize(quat_mul(delta, current));
}

// q_dot = 1/2 q ⊗ ω, with ω expressed in the body frame.
inline Quat quat_derivative(Quat q, double wx, double wy, double wz) {
    Quat omega;
    omega.w = 0.0;
    omega.x = wx;
    omega.y = wy;
    omega.z = wz;
    Quat dq = quat_mul(q, omega);
    dq.w *= 0.5;
    dq.x *= 0.5;
    dq.y *= 0.5;
    dq.z *= 0.5;
    return dq;
}

// Short rotation, in the body frame, that carries the body onto the
// command. Radians. For a small pure roll φ the x component is −φ, so
// a torque kp * error matches the old law τ = −kp φ.
inline void attitude_error_body(Quat q_body, Quat q_cmd, double& ex, double& ey, double& ez) {
    q_body = quat_normalize(q_body);
    q_cmd = quat_normalize(q_cmd);

    Quat body_inv;
    body_inv.w = q_body.w;
    body_inv.x = -q_body.x;
    body_inv.y = -q_body.y;
    body_inv.z = -q_body.z;

    Quat err = quat_mul(body_inv, q_cmd);
    if (err.w < 0.0) {
        err.w = -err.w;
        err.x = -err.x;
        err.y = -err.y;
        err.z = -err.z;
    }

    const double sine = std::sqrt(err.x * err.x + err.y * err.y + err.z * err.z);
    if (sine < 1e-12) {
        ex = 0.0;
        ey = 0.0;
        ez = 0.0;
        return;
    }
    const double angle = 2.0 * std::atan2(sine, err.w);
    const double scale = angle / sine;
    ex = scale * err.x;
    ey = scale * err.y;
    ez = scale * err.z;
}

inline void normalize_state_quaternion(State& state) {
    quat_write(state, quat_normalize(quat_from_state(state)));
}

// q and -q are one attitude. A linear blend has to start on the same
// hemisphere or the short step across touchdown takes the long way around.
inline void same_quaternion_hemisphere(const State& reference, State& state) {
    const double dot =
        reference[QW] * state[QW] +
        reference[QX] * state[QX] +
        reference[QY] * state[QY] +
        reference[QZ] * state[QZ];
    if (dot < 0.0) {
        state[QW] = -state[QW];
        state[QX] = -state[QX];
        state[QY] = -state[QY];
        state[QZ] = -state[QZ];
    }
}

}  // namespace icarus
