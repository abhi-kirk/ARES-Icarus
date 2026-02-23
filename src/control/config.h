#pragma once

#include <string>

namespace icarus {

// PID controller configuration
struct PIDConfig {
    // PID gains for velocity control (inner loop)
    double kp_vel;
    double kd_vel;
    double ki_vel;

    // PID gains for position control (outer loop)
    double kp_pos;
    double kd_pos;
    double ki_pos;
};

// Simulation configuration
struct SimConfig {
    double timestep_inner;
    double timestep_outer;
    double initial_altitude;
    double initial_velocity;
    double initial_fuel;
};

// Complete configuration
struct Config {
    PIDConfig pid;
    SimConfig simulation;
};

// Load configuration from JSON file
// Returns true on success, false on failure
bool load_config(const std::string& filepath, Config& config);

}  // namespace icarus