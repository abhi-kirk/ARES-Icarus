#pragma once

#include "dynamics/constants.h"
#include "integrator.h"

namespace icarus {

// 6-DOF rigid-body landing. Attitude is a quaternion. Gimbal TVC.
// Control: [T, delta_x, delta_y, tau_x, tau_y, tau_z]
//   T          thrust (N)
//   delta_x/y  gimbal angles (rad)
//   tau_x/y/z  extra body torque (N·m), the attitude PD / jets
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
    double gravity = GRAVITY,
    double exhaust_speed = EXHAUST_SPEED,
    bool planet_centered = false
);

// Defaults for inertia and lever when the caller has no case overrides.
State rocket_dynamics_6dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& disturbance
);

}  // namespace icarus
