#include "gnc/control/controller.h"

namespace icarus {

double Controller::compute(double feedback, double setpoint, double feedforward) {
    return pid_step(feedback, setpoint, feedforward, integral_, prev_error_, dt_, spec_);
}

void Controller::reset() {
    integral_   = 0.0;
    prev_error_ = 0.0;
}

}  // namespace icarus
