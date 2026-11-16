#include "dynamics/rocket_6dof.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <cmath>

namespace icarus {

namespace {

void thrust_body(double T, double dx, double dy, double& Fx_b, double& Fy_b, double& Fz_b) {
    const double sdx = std::sin(dx);
    const double cdx = std::cos(dx);
    const double sdy = std::sin(dy);
    const double cdy = std::cos(dy);
    Fx_b = T * sdy;
    Fy_b = T * (-sdx);
    Fz_b = T * cdx * cdy;
}

void body_to_inertial(
    double roll,
    double pitch,
    double yaw,
    double bx,
    double by,
    double bz,
    double& ix,
    double& iy,
    double& iz
) {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    const double r00 = cy * cp;
    const double r01 = cy * sp * sr - sy * cr;
    const double r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp;
    const double r11 = sy * sp * sr + cy * cr;
    const double r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp;
    const double r21 = cp * sr;
    const double r22 = cp * cr;

    ix = r00 * bx + r01 * by + r02 * bz;
    iy = r10 * bx + r11 * by + r12 * bz;
    iz = r20 * bx + r21 * by + r22 * bz;
}

}  // namespace

State rocket_dynamics_6dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& wind,
    double Ixx,
    double Iyy,
    double Izz,
    double gimbal_lever_m
) {
    (void)t;

    const double wind_x = wind.size() > 0 ? wind[0] : 0.0;
    const double wind_y = wind.size() > 1 ? wind[1] : 0.0;
    const double wind_z = wind.size() > 2 ? wind[2] : 0.0;

    const double T  = control.size() > 0 ? control[0] : 0.0;
    const double dx = control.size() > 1 ? control[1] : 0.0;
    const double dy = control.size() > 2 ? control[2] : 0.0;
    const double tau_rcs_x = control.size() > 3 ? control[3] : 0.0;
    const double tau_rcs_y = control.size() > 4 ? control[4] : 0.0;

    double Fx_b = 0.0;
    double Fy_b = 0.0;
    double Fz_b = 0.0;
    thrust_body(T, dx, dy, Fx_b, Fy_b, Fz_b);

    // Gimbal torque: engine at r = [0, 0, -L].
    const double L = gimbal_lever_m;
    const double tau_x = L * Fy_b + tau_rcs_x;
    const double tau_y = -L * Fx_b + tau_rcs_y;
    const double tau_z = 0.0;

    const double roll  = state[ROLL];
    const double pitch = state[PITCH];
    const double yaw   = state[YAW];
    const double wx = state[WX];
    const double wy = state[WY];
    const double wz = state[WZ];

    double Fx_i = 0.0;
    double Fy_i = 0.0;
    double Fz_i = 0.0;
    body_to_inertial(roll, pitch, yaw, Fx_b, Fy_b, Fz_b, Fx_i, Fy_i, Fz_i);

    const double fuel = state[FUEL];
    const double m = MASS_DRY + fuel;

    double m_dot = 0.0;
    if (fuel > 0.0 && T > 0.0) {
        m_dot = -(T / MAX_THRUST) * MAX_FLOW_RATE;
    }

    const double vx_rel = state[VEL_X] - wind_x;
    const double vy_rel = state[VEL_Y] - wind_y;
    const double vz_rel = state[VEL_Z] - wind_z;
    const double speed = std::sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
    const double drag_x = -0.5 * vx_rel * speed;
    const double drag_y = -0.5 * vy_rel * speed;
    const double drag_z = -0.5 * vz_rel * speed;

    const double accel_x = (Fx_i + drag_x) / m;
    const double accel_y = (Fy_i + drag_y) / m;
    const double accel_z = (Fz_i - m * GRAVITY + drag_z) / m;

    const double wx_dot = (tau_x - (Izz - Iyy) * wy * wz) / Ixx;
    const double wy_dot = (tau_y - (Ixx - Izz) * wz * wx) / Iyy;
    const double wz_dot = (tau_z - (Iyy - Ixx) * wx * wy) / Izz;

    const double cos_pitch = std::cos(pitch);
    const double tan_pitch = std::tan(pitch);
    const double sin_roll = std::sin(roll);
    const double cos_roll = std::cos(roll);

    double roll_dot = 0.0;
    double pitch_dot = 0.0;
    double yaw_dot = 0.0;
    if (std::abs(cos_pitch) < 1e-3) {
        roll_dot = wx;
        pitch_dot = wy * cos_roll - wz * sin_roll;
        yaw_dot = 0.0;
    } else {
        roll_dot  = wx + (wy * sin_roll + wz * cos_roll) * tan_pitch;
        pitch_dot = wy * cos_roll - wz * sin_roll;
        yaw_dot   = (wy * sin_roll + wz * cos_roll) / cos_pitch;
    }

    State state_dot(STATE_SIZE);
    state_dot << state[VEL_X], state[VEL_Y], state[VEL_Z],
                 accel_x, accel_y, accel_z,
                 roll_dot, pitch_dot, yaw_dot,
                 wx_dot, wy_dot, wz_dot,
                 m_dot;
    return state_dot;
}

State rocket_dynamics_6dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& disturbance
) {
    return rocket_dynamics_6dof(
        t, state, control, disturbance,
        IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT
    );
}

}  // namespace icarus
