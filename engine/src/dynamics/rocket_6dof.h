#pragma once

#include "dynamics/constants.h"
#include "integrator.h"

namespace icarus {

// 6-DOF rigid-body landing with Euler angles and gimbal TVC.
// Control: [T, delta_x, delta_y, tau_x, tau_y]
//   T          thrust (N)
//   delta_x/y  gimbal angles (rad)
//   tau_x/y    extra body torque (N·m), e.g. attitude PD / RCS
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
    double gravity = GRAVITY
);

// Defaults for inertia and lever when the caller has no case overrides.
State rocket_dynamics_6dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& disturbance
);

}  // namespace icarus
