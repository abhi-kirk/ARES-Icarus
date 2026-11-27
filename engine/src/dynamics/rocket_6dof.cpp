#include "dynamics/rocket_6dof.h"
#include "dynamics/aero.h"
#include "dynamics/attitude.h"
#include "dynamics/constants.h"
#include "dynamics/propulsion.h"
#include "dynamics/state_indices.h"
#include "environment/gravity.h"

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
    double gravity,
    double exhaust_speed,
    bool planet_centered
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
        m_dot = -fuel_flow_kg_s(T_use, exhaust_speed);
    }

    double drag_x = 0.0;
    double drag_y = 0.0;
    double drag_z = 0.0;
    const double height = altitude_m(
        state[POS_X], state[POS_Y], state[POS_Z], planet_centered
    );
    aero_force(
        state[VEL_X], state[VEL_Y], state[VEL_Z],
        wind_x, wind_y, wind_z,
        height, drag_scale, gravity,
        drag_x, drag_y, drag_z
    );

    double grav_x = 0.0;
    double grav_y = 0.0;
    double grav_z = 0.0;
    gravity_accel(
        gravity,
        state[POS_X], state[POS_Y], state[POS_Z],
        planet_centered,
        grav_x, grav_y, grav_z
    );

    const double accel_x = (Fx_i + drag_x) / m + grav_x;
    const double accel_y = (Fy_i + drag_y) / m + grav_y;
    const double accel_z = (Fz_i + drag_z) / m + grav_z;

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
        1.0, GRAVITY, EXHAUST_SPEED, false
    );
}

}  // namespace icarus
