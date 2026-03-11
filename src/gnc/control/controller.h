#pragma once

#include "gnc/control/pid_controller.h"

namespace icarus {

// Stateful wrapper around pid_step. Owns the integral accumulator and
// previous-error state for one control loop.
class Controller {
public:
    Controller(double dt, const PIDSpec& spec) : dt_(dt), spec_(spec) {}

    // feedback and setpoint are raw scalar values (caller extracts from State).
    // feedforward defaults to 0 (e.g., position loop has no gravity compensation).
    double compute(double feedback, double setpoint, double feedforward = 0.0);

    void reset();

private:
    PIDSpec spec_;
    double  integral_   = 0.0;
    double  prev_error_ = 0.0;
    double  dt_;
};

}  // namespace icarus
