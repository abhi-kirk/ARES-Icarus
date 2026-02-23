#include <filesystem>
#include <fstream>
#include <iostream>

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
    csv_file << "Time,Altitude,Velocity,Fuel,Throttle" << endl;

    cout << "Time, Altitude, Velocity, Fuel" << endl;

    Controller vel_controller(dt_inner, config.pid);
    Controller pos_controller(dt_outer, config.pid);

    int outer_loop_divider = 50;  // Outer loop runs 50x slower
    int iteration = 0;
    double ref_velocity = 0.0;

    while (state[POS] > 0.0) {
        if (iteration % outer_loop_divider == 0) {
            // Control Logic (Position feedback -> reference velocity)
            ref_velocity = pos_controller.compute(state, "position", 0.0);
        }

        // Control Logic (Velocity feedback -> throttle)
        double throttle = vel_controller.compute(state, "velocity", ref_velocity);

        // Physics
        // Bind the throttle to the dynamics function using a lambda
        // throttle is permenently set, and time, state are passed to bound_dynamics
        SystemDynamics bound_dynamics = [throttle](double time, const State& s) {
            return rocket_dynamics(time, s, throttle);
        };

        // Use RK4 to integrate (state prediction)
        state = Integrator::rk4_step(bound_dynamics, t, state, dt_inner);

        t += dt_inner;

        // Logging
        cout << t << ", " << state[POS] << ", " << state[VEL] << ", " << state[FUEL] << endl;
        csv_file << t << "," << state[POS] << "," << state[VEL] << "," << state[FUEL] << ","
                 << throttle << endl;

        if (iteration % 10 == 0) {
            cout << "t=" << t << " alt=" << state[POS] << " vel=" << state[VEL]
                 << " ref_vel=" << ref_velocity << " throttle=" << throttle << endl;
        }

        iteration++;
    }

    cout << "Landed/Crashed at Velocity: " << state[VEL] << "m/s" << endl;
    csv_file.close();
    return 0;
}