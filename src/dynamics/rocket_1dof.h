#pragma once

#include "integrator.h"

namespace icarus {

// 1-DOF rocket dynamics: vertical landing with thrust, gravity, and quadratic drag.
// Returns state_dot = [velocity, acceleration, fuel_dot].
State rocket_dynamics(double t, const State& state, double throttle);

}  // namespace icarus
