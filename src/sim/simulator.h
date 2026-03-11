#pragma once

#include <functional>

#include "config/config.h"
#include "gnc/control/controller.h"
#include "gnc/guidance/cascade_guidance.h"
#include "integrator.h"
#include "sim/logger.h"

namespace icarus {

// f(t, state, throttle) -> state_dot
using ThrottledDynamics = std::function<State(double, const State&, double)>;

class Simulator {
public:
    // Runs the simulation loop until altitude reaches zero.
    // state is modified in place; t_start is the initial time.
    void run(State& state, double t_start,
             const SimConfig& cfg,
             CascadeGuidance& guidance,
             Controller& vel_ctrl,
             const ThrottledDynamics& dynamics,
             Logger& logger);
};

}  // namespace icarus
