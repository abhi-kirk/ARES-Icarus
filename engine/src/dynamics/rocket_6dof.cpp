#include "dynamics/rocket_6dof.h"
#include "dynamics/attitude.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <algorithm>
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

}  // namespace

State rocket_dynamics_6dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& wind,
    double Ixx,
    double Iyy,
    double Izz,
    double gimbal_lever_m,
    double drag_scale,
    double gravity
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
    const double tau_rcs_z = control.size() > 5 ? control[5] : 0.0;

    const double fuel = state[FUEL];
    // An empty tank produces no thrust and no gimbal torque. RCS still acts.
    const double T_use = (fuel > 0.0) ? T : 0.0;
    double Fx_b = 0.0;
    double Fy_b = 0.0;
    double Fz_b = 0.0;
    thrust_body(T_use, dx, dy, Fx_b, Fy_b, Fz_b);

    // Gimbal torque: engine at r = [0, 0, -L].
    const double L = gimbal_lever_m;
    const double tau_x = L * Fy_b + tau_rcs_x;
    const double tau_y = -L * Fx_b + tau_rcs_y;
    const double tau_z = tau_rcs_z;

    const Quat q = quat_normalize(quat_from_state(state));
    const double wx = state[WX];
    const double wy = state[WY];
    const double wz = state[WZ];

    double Fx_i = 0.0;
    double Fy_i = 0.0;
    double Fz_i = 0.0;
    rotate_body_to_inertial(q, Fx_b, Fy_b, Fz_b, Fx_i, Fy_i, Fz_i);

    const double m = MASS_DRY + std::max(0.0, fuel);

    double m_dot = 0.0;
    if (fuel > 0.0 && T_use > 0.0) {
        m_dot = -(T_use / MAX_THRUST) * MAX_FLOW_RATE;
    }

    const double vx_rel = state[VEL_X] - wind_x;
    const double vy_rel = state[VEL_Y] - wind_y;
    const double vz_rel = state[VEL_Z] - wind_z;
    const double speed = std::sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
    // drag_scale = 1 keeps the historical law. 0 is vacuum.
    const double drag_coeff = 0.5 * std::max(0.0, drag_scale);
    const double drag_x = -drag_coeff * vx_rel * speed;
    const double drag_y = -drag_coeff * vy_rel * speed;
    const double drag_z = -drag_coeff * vz_rel * speed;

    const double accel_x = (Fx_i + drag_x) / m;
    const double accel_y = (Fy_i + drag_y) / m;
    const double accel_z = (Fz_i - m * gravity + drag_z) / m;

    const double wx_dot = (tau_x - (Izz - Iyy) * wy * wz) / Ixx;
    const double wy_dot = (tau_y - (Ixx - Izz) * wz * wx) / Iyy;
    const double wz_dot = (tau_z - (Iyy - Ixx) * wx * wy) / Izz;

    const Quat q_dot = quat_derivative(q, wx, wy, wz);

    State state_dot = State::Zero(STATE_SIZE);
    state_dot[POS_X] = state[VEL_X];
    state_dot[POS_Y] = state[VEL_Y];
    state_dot[POS_Z] = state[VEL_Z];
    state_dot[VEL_X] = accel_x;
    state_dot[VEL_Y] = accel_y;
    state_dot[VEL_Z] = accel_z;
    state_dot[QW] = q_dot.w;
    state_dot[QX] = q_dot.x;
    state_dot[QY] = q_dot.y;
    state_dot[QZ] = q_dot.z;
    state_dot[WX] = wx_dot;
    state_dot[WY] = wy_dot;
    state_dot[WZ] = wz_dot;
    state_dot[FUEL] = m_dot;
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
        IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT,
        1.0, GRAVITY
    );
}

}  // namespace icarus
