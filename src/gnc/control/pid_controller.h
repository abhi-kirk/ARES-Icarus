#pragma once

namespace icarus {

// Configuration for a single PID loop.
struct PIDSpec {
    double kp          = 0.0;
    double ki          = 0.0;
    double kd          = 0.0;
    double control_min = 0.0;
    double control_max = 1.0;
};

// Single PID step. Pure function — no knowledge of state vectors or physics.
// Modifies integral and prev_error in place (caller owns the state).
// feedforward is added after PID sum, before clamping.
double pid_step(double feedback, double setpoint, double feedforward,
                double& integral, double& prev_error,
                double dt, const PIDSpec& spec);

}  // namespace icarus
