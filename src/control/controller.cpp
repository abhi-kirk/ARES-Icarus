#include "controller.h"

namespace icarus {

double Controller::compute(const State& state, const string& measurement_type,
                           const double& setpoint) {
    return pid_controller(state, measurement_type, setpoint, integral_feedback_error_,
                          previous_feedback_error_, dt_, config_);
}
}  // namespace icarus