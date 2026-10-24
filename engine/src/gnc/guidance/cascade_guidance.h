#pragma once

#include "config/config.h"
#include "gnc/control/controller.h"
#include "integrator.h"

namespace icarus {

// Outer vertical loop of the cascade PID architecture.
// Vertical position error -> rate-limited vertical reference velocity command.
// The caller (Simulator) feeds ref_vel_cmd into the vertical velocity controller.
class CascadeGuidance {
public:
    explicit CascadeGuidance(const Config& config);

    // Call once before the simulation loop to seed ref_vel_cmd from the current
    // position controller output, preventing a startup ramp artifact.
    void initialize(const State& state);

    // Returns the rate-limited commanded reference velocity for this timestep.
    double compute(const State& state);

    double ref_vel_target() const { return ref_vel_target_; }

private:
    Controller pos_ctrl_;
    double ref_vel_target_ = 0.0;
    double ref_vel_cmd_    = 0.0;
    double max_ref_accel_;
    double dt_inner_;
    int    outer_loop_divider_;
    int    iteration_ = 0;
};

}  // namespace icarus
