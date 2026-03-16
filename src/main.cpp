#include <filesystem>
#include <iostream>

#include "config/config.h"
#include "dynamics/constants.h"
#include "dynamics/rocket_1dof.h"
#include "dynamics/state_indices.h"
#include "gnc/control/controller.h"
#include "gnc/control/pid_controller.h"
#include "gnc/guidance/cascade_guidance.h"
#include "sim/logger.h"
#include "sim/simulator.h"

using namespace icarus;

int main() {
    Config config;
    const auto exe_path = std::filesystem::current_path();
    if (!load_config((exe_path / "config.json").string(), config)) return 1;

    State state(3);
    state << config.simulation.initial_altitude,
             config.simulation.initial_velocity,
             config.simulation.initial_fuel;

    // Guidance: outer loop (position -> rate-limited ref_velocity)
    CascadeGuidance guidance(config);
    guidance.initialize(state);

    // Velocity controller: inner loop (velocity -> throttle)
    const PIDSpec vel_spec{
        config.pid.kp_vel, config.pid.ki_vel, config.pid.kd_vel,
        MIN_THROTTLE, MAX_THROTTLE
    };
    Controller vel_ctrl(config.simulation.timestep_inner, vel_spec);

    const auto output_path = (exe_path / "src" / "results" / "trajectory.csv").string();
    const auto rrd_path    = (exe_path / "src" / "results" / "trajectory.rrd").string();
    Logger logger(output_path,
                  config.simulation.use_rerun,
                  config.simulation.rerun_spawn_viewer,
                  config.simulation.rerun_save_rrd,
                  rrd_path);

    Simulator sim;
    sim.run(state, 0.0, config.simulation, guidance, vel_ctrl, rocket_dynamics, logger);

    std::cout << "Landed at velocity: " << state[VEL] << " m/s\n";
    return 0;
}
