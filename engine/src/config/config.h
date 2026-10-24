#pragma once

#include <string>

namespace icarus {

struct PIDConfig {
    // Vertical inner loop: velocity -> force
    double kp_v_vel, ki_v_vel, kd_v_vel;

    // Vertical outer loop: altitude -> ref_vel
    double kp_v_pos, ki_v_pos, kd_v_pos;

    // Horizontal inner loop: velocity -> force (same gains for x and y)
    double kp_h_vel, ki_h_vel, kd_h_vel;

    // Horizontal outer loop: position error -> ref_vel (same gains for x and y)
    double kp_h_pos, ki_h_pos, kd_h_pos;
};

struct SimConfig {
    double timestep_inner;
    double timestep_outer;
    double timestep_inner_h;
    double timestep_outer_h;


    double max_ref_accel_v;
    double max_ref_accel_h;

    double initial_px;
    double initial_py;
    double initial_pz;

    double initial_vx;
    double initial_vy;
    double initial_vz;

    double initial_fuel;

    double wind_constant_x;
    double wind_constant_y;
    double gust_magnitude;
    double gust_frequency;

    double pad_initial_x;
    double pad_initial_y;
    double pad_drift_vx;
    double pad_drift_vy;

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
