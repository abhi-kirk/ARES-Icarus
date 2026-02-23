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

    // Integral term: accumulate feedback error over time
    integral_feedback_error += feedback_error * dt;

    // Derivative term: rate of change of feedback error
    const double derivative_feedback_error = (feedback_error - previous_feedback_error) / dt;

    // Update previous feedback error for next iteration
    previous_feedback_error = feedback_error;

    // PID control law
    const double proportional_term = kp * feedback_error;
    const double integral_term = ki * integral_feedback_error;
    const double derivative_term = kd * derivative_feedback_error;

    const double pid_output = proportional_term + integral_term + derivative_term + feedforward_term;

    // Total control output (clamped to min/max)
    const double control_output = std::clamp(pid_output + feedforward_term, control_min, control_max);

    return control_output;
}
}  // namespace icarus