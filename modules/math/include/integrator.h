#pragma once
#include <Eigen/Dense>
#include <functional>

using std::function;

namespace icarus {
    // Define a "State" as a vector of doubles
    using State = Eigen::VectorXd;

    // Define the "System Function" signatures: f(t, y) -> y_dot
    using SystemDynamics = function<State(double t, const State& y)>;

    class Integrator {
    public:
    // The magic function: Returns the state at t+dt
    static State rk4_step(SystemDynamics f, double t, const State& y, double dt);
    };
}  // namespace icarus