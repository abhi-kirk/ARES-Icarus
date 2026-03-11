#include <algorithm>

#include "config.h"
#include "constants.h"
#include "integrator.h"
#include "state_indices.h"

using namespace icarus;
using std::string;

namespace icarus {

double pid_controller(const State& state, const string& measurement_type, const double& setpoint,
                      double& integral_feedback_error, double& previous_feedback_error, double dt,
                      const PIDConfig& config) {
    // Determine controller/measurement type
    double feedback = 0.0;
    double kp, kd, ki;
    kp = kd = ki = 0.0;
    double feedforward_term = 0.0;
    double control_min = 0.0, control_max = 0.0;

    if (measurement_type == "velocity") {
        feedback = state[VEL];
        kp = config.kp_vel;
        kd = config.kd_vel;
        ki = config.ki_vel;
        feedforward_term = (MASS_DRY + state[FUEL]) * GRAVITY /
                           MAX_THRUST;  // hover throttle to counteract gravity
        control_min = MIN_THROTTLE;
        control_max = MAX_THROTTLE;
    } else if (measurement_type == "position") {
        feedback = state[POS];
        kp = config.kp_pos;
        kd = config.kd_pos;
        ki = config.ki_pos;
        control_min = -20;
        control_max = 0.0;
    }

    // Calculate feedback error (setpoint - actual)
    const double feedback_error = setpoint - feedback;

    // Derivative term: rate of change of feedback error
    const double derivative_feedback_error = (feedback_error - previous_feedback_error) / dt;

    // Update previous feedback error for next iteration
    previous_feedback_error = feedback_error;

    // PID control law (compute unclamped output first so we can do anti-windup)
    const double proportional_term = kp * feedback_error;
    const double proposed_integral_feedback_error = integral_feedback_error + (feedback_error * dt);
    const double integral_term = ki * proposed_integral_feedback_error;
    const double derivative_term = kd * derivative_feedback_error;

    const double pid_output = proportional_term + integral_term + derivative_term;

    // Total control output (clamped to min/max)
    const double unclamped_output = pid_output + feedforward_term;
    const double control_output = std::clamp(unclamped_output, control_min, control_max);

    // Anti-windup: if we're saturated and the error would drive us further into saturation,
    // don't integrate this step. Otherwise, accept the integral update.
    //
    // This is "conditional integration" and works for both velocity and position loops
    // because the controller output is monotonically increasing with feedback_error.
    const bool is_saturated = (control_output != unclamped_output);
    if (!is_saturated) {
        integral_feedback_error = proposed_integral_feedback_error;
    } else {
        const bool saturating_high = (control_output >= control_max);
        const bool saturating_low = (control_output <= control_min);
        const bool error_pushes_high = (feedback_error > 0.0);
        const bool error_pushes_low = (feedback_error < 0.0);

        const bool allow_integrate =
            (saturating_high && !error_pushes_high) || (saturating_low && !error_pushes_low);

        if (allow_integrate) {
            integral_feedback_error = proposed_integral_feedback_error;
        }
    }

    return control_output;
}
}  // namespace icarus
