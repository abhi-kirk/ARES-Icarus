#pragma once

#include <string>

namespace icarus {

struct PIDConfig {
    double kp_vel;
    double kd_vel;
    double ki_vel;

    double kp_pos;
    double kd_pos;
    double ki_pos;
};

struct SimConfig {
    double timestep_inner;
    double timestep_outer;
    double max_ref_accel;
    double initial_altitude;
    double initial_velocity;
    double initial_fuel;

    bool use_rerun          = false;
    bool rerun_spawn_viewer = true;
    bool rerun_save_rrd     = true;
};

struct Config {
    PIDConfig pid;
    SimConfig simulation;
};

bool load_config(const std::string& filepath, Config& config);

}  // namespace icarus
