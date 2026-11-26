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

    // Actuator bounds on the assembled thrust vector [Fx, Fy, Fz].
    // thrust_min / thrust_max are newtons. pointing_cone_deg is the max
    // tilt from vertical (90 means the cone does not cut).
    double thrust_min = 0.0;
    double thrust_max = 25000.0;
    double pointing_cone_deg = 90.0;

    // Approach corridor from the pad: horizontal range may sit no farther
    // than height * tan(glide_cone_deg). 90 means guidance ignores the wall.
    // glide_pull is extra inward ref velocity (m/s) per meter outside the wall.
    double glide_cone_deg = 90.0;
    double glide_pull = 0.5;

    // Engine response. thrust_tau is the first-order lag time (s); 0 means
    // the force jumps with the command. thrust_slew is an optional hard
    // rate limit in N/s; 0 means no extra slew cap beyond the lag.
    double thrust_tau = 0.0;
    double thrust_slew = 0.0;

    // Multiplier on the quadratic drag law. 1 is the historical plant.
    // 0 is vacuum. Wind is a separate field.
    double drag_scale = 1.0;

    // Surface gravity, m/s^2. 9.81 is Earth. 1.62 is the Moon. The thrust
    // band has to be scaled with it or the floor sits above lunar weight.
    double gravity = 9.81;

    // 6-DOF plant. When use_6dof is true the stream integrates a quaternion
    // attitude with gimbal TVC; the force PID still sets the ask. Samples
    // still report roll, pitch, and yaw derived from that quaternion.
    bool use_6dof = true;
    double Ixx = 2000.0;
    double Iyy = 2000.0;
    double Izz = 400.0;
    double gimbal_lever = 8.0;
    double gimbal_max_deg = 15.0;
    double attitude_kp = 1.5;
    double attitude_kd = 0.8;

    // Guidance law for the force ask. "pid" is the cascade. "mpc" is LTV-MPC
    // (Phase 5) that tracks a soft-landing reference with OSQP.
    std::string controller = "pid";

    // MPC horizon length (steps) and step size (s). Weights are diagonal
    // costs on position, velocity, and force effort.
    int mpc_horizon = 50;
    double mpc_dt = 0.1;
    double mpc_w_pos = 40.0;
    double mpc_w_vel = 50.0;
    double mpc_w_u = 1.0e-9;
    double mpc_w_terminal = 120.0;

    bool use_rerun          = false;
    bool rerun_spawn_viewer = true;
    bool rerun_save_rrd     = true;
};

struct Config {
    PIDConfig pid;
    SimConfig simulation;
};

bool load_config(const std::string& filepath, Config& config);

// Same fields as load_config, read from a JSON string. Writes nothing on success,
// so a caller can keep stdout for flight samples.
bool load_config_text(const std::string& text, Config& config);

}  // namespace icarus
