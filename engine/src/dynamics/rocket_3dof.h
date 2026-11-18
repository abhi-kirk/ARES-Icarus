#pragma once

#include "dynamics/constants.h"
#include "integrator.h"

namespace icarus {

// 3-DOF rocket dynamics
State rocket_dynamics_3dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& disturbance,
    double drag_scale,
    double gravity = GRAVITY
);

}  // namespace icarus