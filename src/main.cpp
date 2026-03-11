#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "control/config.h"
#include "control/controller.h"
#include "dynamics/constants.h"
#include "integrator.h"

using namespace icarus;

using std::abs;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;

// State Index Mapping
enum {
    POS = 0,   // Altitude
    VEL = 1,   // Velocity
    FUEL = 2,  // Fuel Mass
};

// Output File
std::filesystem::path exe_path = std::filesystem::current_path();
std::string output_file = (exe_path / "src" / "results" / "trajectory.csv").string();

static double rate_limit(double current, double target, double max_rate, double dt) {
    if (max_rate <= 0.0 || dt <= 0.0) {
        return target;
    }
    const double max_delta = max_rate * dt;
    const double delta = target - current;
    return current + std::clamp(delta, -max_delta, max_delta);
}

// The System Dynamics function (The Derivative)
State rocket_dynamics(double t, const State& state, double throttle) {
    (void)t;  // Unused for now (autonomous system)

    double y = state[POS];
    double v = state[VEL];
    double fuel = state[FUEL];
    double m = MASS_DRY + fuel;

    // Forces
    double thrust = 0.0;
    double m_dot = 0.0;

    if (fuel > 0) {
        thrust = throttle * MAX_THRUST;
        m_dot = -throttle * FLOW_RATE;  // equal to fuel_dot
    }

    // Drag (Simple quadratic: 0.5 * rho * v^2 * Cd * A)
    // Approximate drag coefficient K = 0.5
    double drag = -0.5 * v * abs(v);
    // Note: drag opposes velocity. If v is negative (falling), drag is up (positive)

    double net_force = thrust - (m * GRAVITY) - drag;
    double acceleration = net_force / m;

    State state_dot(3);
    state_dot << v, acceleration, m_dot;
    return state_dot;
}

int main() {
    // Load configuration
    Config config;
    std::filesystem::path config_path = exe_path / "config.json";
    if (!load_config(config_path.string(), config)) {
        std::cerr << "Failed to load config. Using default values." << std::endl;
        return 1;
    }

    // Initial Conditions from config
    State state(3);
    state << config.simulation.initial_altitude, config.simulation.initial_velocity,
        config.simulation.initial_fuel;

    double t = 0.0;
    double dt_inner = config.simulation.timestep_inner;
    double dt_outer = config.simulation.timestep_outer;

    ofstream csv_file(output_file);  // save results for python plots
    csv_file << std::fixed << std::setprecision(6);
    csv_file << "Time,Altitude,Velocity,Fuel,Throttle" << endl;

    cout << "Time, Altitude, Velocity, Fuel" << endl;

    Controller vel_controller(dt_inner, config.pid);
    Controller pos_controller(dt_outer, config.pid);

    // Outer loop runs slower (ensure a valid divider even if dt_outer isn't an exact multiple)
    int outer_loop_divider = static_cast<int>(std::lround(dt_outer / dt_inner));
    outer_loop_divider = std::max(1, outer_loop_divider);
    int iteration = 0;
    double ref_velocity_target = 0.0;
    double ref_velocity_cmd = 0.0;

    // Initialize the command to the current outer-loop target to avoid an initial setpoint ramp
    // artifact in plots when starting in a terminal-descent condition.
    ref_velocity_target = pos_controller.compute(state, "position", 0.0);
    ref_velocity_cmd = ref_velocity_target;

    while (state[POS] > 0.0) {
        const State prev_state = state;
        const double prev_t = t;
        if (iteration % outer_loop_divider == 0) {
            // Control Logic (Position feedback -> reference velocity)
            ref_velocity_target = pos_controller.compute(state, "position", 0.0);
        }

        // Rate-limit the commanded reference velocity (acceleration-limited command shaping)
        ref_velocity_cmd = rate_limit(ref_velocity_cmd, ref_velocity_target,
                                      config.simulation.max_ref_accel, dt_inner);

        // Control Logic (Velocity feedback -> throttle)
        double throttle = vel_controller.compute(state, "velocity", ref_velocity_cmd);

        // Physics
        // Bind the throttle to the dynamics function using a lambda
        // throttle is permenently set, and time, state are passed to bound_dynamics
        SystemDynamics bound_dynamics = [throttle](double time, const State& s) {
            return rocket_dynamics(time, s, throttle);
        };

        // Use RK4 to integrate (state prediction)
        state = Integrator::rk4_step(bound_dynamics, t, state, dt_inner);

        t += dt_inner;

        // If we crossed the ground plane this step, interpolate to touchdown so
        // the final logged velocity matches the contact event (not an overshoot).
        if (state[POS] <= 0.0) {
            const double y0 = prev_state[POS];
            const double y1 = state[POS];
            const double denom = y0 - y1;
            double alpha = 1.0;
            if (denom > 0.0 && y0 > 0.0) {
                alpha = std::clamp(y0 / denom, 0.0, 1.0);
            }

            const double touchdown_t = prev_t + alpha * dt_inner;
            const double touchdown_v = prev_state[VEL] + alpha * (state[VEL] - prev_state[VEL]);
            const double touchdown_fuel = prev_state[FUEL] + alpha * (state[FUEL] - prev_state[FUEL]);

            t = touchdown_t;
            state[POS] = 0.0;
            state[VEL] = touchdown_v;
            state[FUEL] = std::max(0.0, touchdown_fuel);
        }

        // Clamp fuel after touchdown interpolation (and for normal steps) so we never log negative fuel.
        if (state[FUEL] < 0.0) {
            state[FUEL] = 0.0;
        }

        // Logging
        cout << t << ", " << state[POS] << ", " << state[VEL] << ", " << state[FUEL] << endl;
        csv_file << t << "," << state[POS] << "," << state[VEL] << "," << state[FUEL] << ","
                 << throttle << endl;

        if (iteration % 10 == 0) {
            cout << "t=" << t << " alt=" << state[POS] << " vel=" << state[VEL]
                 << " ref_vel_cmd=" << ref_velocity_cmd << " ref_vel_target=" << ref_velocity_target
                 << " throttle=" << throttle << endl;
        }

        iteration++;
    }

    cout << "Landed/Crashed at Velocity: " << state[VEL] << "m/s" << endl;
    csv_file.close();
    return 0;
}
