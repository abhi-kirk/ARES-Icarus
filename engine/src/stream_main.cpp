// Headless landing. A case JSON arrives on stdin. Each sample leaves on stdout
// as one JSON line, while the rocket is still flying. SIGTERM stops the loop.
// Nothing else is written to stdout, so a caller can read the stream directly.

#include <atomic>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>

#include "config/config.h"
#include "dynamics/constants.h"
#include "dynamics/rocket_3dof.h"
#include "dynamics/rocket_6dof.h"
#include "dynamics/state_indices.h"
#include "environment/landing_pad.h"
#include "environment/wind_model.h"
#include "gnc/control/controller.h"
#include "gnc/control/pid_controller.h"
#include "gnc/guidance/cascade_guidance.h"
#include "gnc/guidance/horizontal_guidance.h"
#include "sim/simulator.h"

using namespace icarus;

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int) {
    g_stop.store(true);
}

// A landing that never reaches the ground still has to end.
const double TIME_LIMIT = 120.0;

void write_sample(const FlightSample& sample) {
    std::cout << "{\"t\":" << sample.t
              << ",\"x\":" << sample.x
              << ",\"y\":" << sample.y
              << ",\"z\":" << sample.z
              << ",\"vx\":" << sample.vx
              << ",\"vy\":" << sample.vy
              << ",\"vz\":" << sample.vz
              << ",\"fuel\":" << sample.fuel
              << ",\"rho\":" << sample.rho
              << ",\"fx\":" << sample.fx
              << ",\"fy\":" << sample.fy
              << ",\"fz\":" << sample.fz
              << ",\"padX\":" << sample.pad_x
              << ",\"padY\":" << sample.pad_y
              << ",\"roll\":" << sample.roll
              << ",\"pitch\":" << sample.pitch
              << ",\"yaw\":" << sample.yaw
              << ",\"gimbalX\":" << sample.gimbal_x
              << ",\"gimbalY\":" << sample.gimbal_y;
    if (sample.plan_valid) {
        std::cout << ",\"planX\":" << sample.plan_x
                  << ",\"planY\":" << sample.plan_y
                  << ",\"planZ\":" << sample.plan_z
                  << ",\"planTrail\":[";
        for (size_t i = 0; i < sample.plan_trail.size(); ++i) {
            if (i > 0) std::cout << ",";
            std::cout << sample.plan_trail[i];
        }
        std::cout << "]";
    }
    std::cout << "}\n";
    std::cout.flush();
    if (!std::cout) g_stop.store(true);
}

}  // namespace

int main() {
    std::signal(SIGTERM, on_signal);
    std::signal(SIGINT, on_signal);

    std::ostringstream text;
    text << std::cin.rdbuf();

    Config config;
    if (!load_config_text(text.str(), config)) return 1;

    State state = State::Zero(STATE_SIZE);
    state[QW] = 1.0;
    state[POS_X] = config.simulation.initial_px;
    state[POS_Y] = config.simulation.initial_py;
    state[POS_Z] = config.simulation.initial_pz;
    state[VEL_X] = config.simulation.initial_vx;
    state[VEL_Y] = config.simulation.initial_vy;
    state[VEL_Z] = config.simulation.initial_vz;
    state[FUEL]  = config.simulation.initial_fuel;

    CascadeGuidance guidance(config);
    guidance.initialize(state);

    Eigen::Vector2d pad_position_init = {config.simulation.pad_initial_x, config.simulation.pad_initial_y};
    Eigen::Vector2d pad_velocity_init = {config.simulation.pad_drift_vx, config.simulation.pad_drift_vy};

    HorizontalGuidance guidance_h(config);
    guidance_h.initialize(state, pad_position_init, pad_velocity_init);

    // Scale the normalized gains by the case thrust ceiling so the PID
    // saturates near the same place the thrust clamp does.
    const double thrust_ceil = config.simulation.thrust_max;
    const PIDSpec vel_spec{
        config.pid.kp_v_vel * thrust_ceil,
        config.pid.ki_v_vel * thrust_ceil,
        config.pid.kd_v_vel * thrust_ceil,
        0.0, thrust_ceil
    };
    Controller vel_ctrl(config.simulation.timestep_inner, vel_spec);

    const PIDSpec vel_spec_h{
        config.pid.kp_h_vel * thrust_ceil,
        config.pid.ki_h_vel * thrust_ceil,
        config.pid.kd_h_vel * thrust_ceil,
        -thrust_ceil, thrust_ceil
    };
    Controller vel_ctrl_h_x(config.simulation.timestep_inner_h, vel_spec_h);
    Controller vel_ctrl_h_y(config.simulation.timestep_inner_h, vel_spec_h);

    WindModel wind_model(config.simulation);
    LandingPad landing_pad(config.simulation);

    SampleEmit emit = write_sample;
    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);

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
        [&](double time, const State& s, const ControlInput& u, const Disturbance& d) {
            return rocket_dynamics_3dof(
                time, s, u, d, config.simulation.drag_scale, config.simulation.gravity
            );
        },
        wind_model,
        landing_pad,
        nullptr,
        &g_stop,
        &emit,
        config.simulation.entry ? 400.0 : TIME_LIMIT
    );

    return 0;
}
