#include "gnc/control/pid_controller.h"

#include <algorithm>

namespace icarus {

double pid_step(double feedback, double setpoint, double feedforward,
                double& integral, double& prev_error,
                double dt, const PIDSpec& spec) {
    const double error = setpoint - feedback;
    const double deriv = (error - prev_error) / dt;
    prev_error = error;

    // Propose integral update before clamping so we can apply anti-windup.
    const double proposed_integral = integral + error * dt;

    const double p       = spec.kp * error;
    const double i       = spec.ki * proposed_integral;
    const double d       = spec.kd * deriv;
    const double unclamped = p + i + d + feedforward;
    const double output    = std::clamp(unclamped, spec.control_min, spec.control_max);

    // Conditional integration (anti-windup): only accept the integral update when
    // we are not saturated, or when the error would drive us back into the linear region.
    const bool saturated = (output != unclamped);
    if (!saturated) {
        integral = proposed_integral;
    } else {
        const bool sat_high  = (output >= spec.control_max);
        const bool sat_low   = (output <= spec.control_min);
        const bool err_high  = (error > 0.0);
        const bool err_low   = (error < 0.0);
        if ((sat_high && !err_high) || (sat_low && !err_low)) {
            integral = proposed_integral;
        }
    }

    return output;
}

}  // namespace icarus
