#pragma once

#include <functional>

#include "config/config.h"
#include "gnc/control/controller.h"
#include "gnc/guidance/cascade_guidance.h"
#include "gnc/guidance/horizontal_guidance.h"
#include "environment/wind_model.h"
#include "environment/landing_pad.h"
#include "integrator.h"
#include "sim/logger.h"

namespace icarus {

// f(t, state, control inputs, external disturbances) -> state_dot
using DynamicsFunction = std::function<State(double, const State&, const ControlInput&, const Disturbance&)>;

class Simulator {
public:
    // Runs the simulation loop until altitude reaches zero.
    // state is modified in place; t_start is the initial time.
    void run(
        State& state,
        double t_start,
        const SimConfig& cfg,
        CascadeGuidance& guidance,
        HorizontalGuidance& guidance_h,
        Controller& vel_ctrl,  // vertical velocity controller for inner loop
        Controller& vel_ctrl_h_x,  // x-coordinate horizontal velocity controller for inner loop
        Controller& vel_ctrl_h_y,  // y-coordinate horizontal velocity controller for inner loop
        const DynamicsFunction& dynamics,
        const WindModel& wind_model,
        const LandingPad& landing_pad,
        Logger& logger
    );
};

}  // namespace icarus
