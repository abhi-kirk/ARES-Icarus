#include <filesystem>
#include <iostream>

#include "config/config.h"
#include "dynamics/constants.h"
#include "dynamics/rocket_3dof.h"
#include "dynamics/state_indices.h"
#include "gnc/control/controller.h"
#include "gnc/control/pid_controller.h"
#include "gnc/guidance/cascade_guidance.h"
#include "gnc/guidance/horizontal_guidance.h"
#include "environment/wind_model.h"
#include "environment/landing_pad.h"
#include "sim/logger.h"
#include "sim/simulator.h"

using namespace icarus;

int main() {
    Config config;
    const auto exe_path = std::filesystem::current_path();
    if (!load_config((exe_path / "config.json").string(), config)) return 1;

    State state(STATE_SIZE);
    state << config.simulation.initial_px,
             config.simulation.initial_py,
             config.simulation.initial_pz,
             config.simulation.initial_vx,
             config.simulation.initial_vy,
             config.simulation.initial_vz,
             config.simulation.initial_fuel;

    // Guidance: outer vertical loop (position -> rate-limited ref_velocity)
    CascadeGuidance guidance(config);
    guidance.initialize(state);

    // Pad initial state
    Eigen::Vector2d pad_position_init = {config.simulation.pad_initial_x, config.simulation.pad_initial_y};
    Eigen::Vector2d pad_velocity_init = {config.simulation.pad_drift_vx, config.simulation.pad_drift_vy};

    // Guidance: outer horizontal loop
    HorizontalGuidance guidance_h(config);
    guidance_h.initialize(state, pad_position_init, pad_velocity_init);

    // Vertical velocity controller: inner loop (velocity -> force/thrust)
    const PIDSpec vel_spec{
        config.pid.kp_v_vel * MAX_THRUST,
        config.pid.ki_v_vel * MAX_THRUST,
        config.pid.kd_v_vel * MAX_THRUST,
        0.0, MAX_THRUST
    };
    Controller vel_ctrl(config.simulation.timestep_inner, vel_spec);

    // Horizontal velocity controller
    const PIDSpec vel_spec_h{
        config.pid.kp_h_vel * MAX_THRUST,
        config.pid.ki_h_vel * MAX_THRUST,
        config.pid.kd_h_vel * MAX_THRUST,
        -MAX_THRUST, MAX_THRUST
    };
    Controller vel_ctrl_h_x(config.simulation.timestep_inner_h, vel_spec_h);
    Controller vel_ctrl_h_y(config.simulation.timestep_inner_h, vel_spec_h);

    // Wind model
    WindModel wind_model(config.simulation);

    // Landing pad
    LandingPad landing_pad(config.simulation);

    const auto output_path = (exe_path / "src" / "results" / "trajectory.csv").string();
    const auto rrd_path    = (exe_path / "src" / "results" / "trajectory.rrd").string();
    Logger logger(output_path,
                  config.simulation.use_rerun,
                  config.simulation.rerun_spawn_viewer,
                  config.simulation.rerun_save_rrd,
                  rrd_path);

    Simulator sim;
    sim.run(
        state,
        0.0,
        config.simulation,
        guidance,
        guidance_h,
        vel_ctrl,
        vel_ctrl_h_x,
        vel_ctrl_h_y,
        rocket_dynamics_3dof,
        wind_model,
        landing_pad,
        logger
    );

    std::cout << "Landed at velocity: " << state[VEL_Z] << " m/s\n";
    return 0;
}
