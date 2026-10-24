#include "integrator.h"

#include <cstddef>

namespace icarus {

// Runge–Kutta 4 Implementation
State Integrator::rk4_step(SystemDynamics f, double t, const State& y, double dt) {
    // k1 = f(t, y);
    // Euler step (not taken, but noted), where k1 is the slope and the step would be k1
    State k1 = f(t, y);

    // k2 = f(t + dt/2, y + dt*k1/2)
    // Starting from y, mid-point of Euler (k1) slope: k1/2 * dt
    // Noted slope here: k2
    State k2 = f(t + 0.5 * dt, y + (k1 * 0.5 * dt));

    // k3 = f(t + dt/2, y + dt*k2/2)
    // Starting from y, mid-point on k2 slope: k2/2 * dt
    // Noted slope here: k3
    State k3 = f(t + 0.5 * dt, y + (k2 * 0.5 * dt));

    // k4 = f(t + dt, y + dt*k3)
    // Starting from y, full step on k3 slope: k3 * dt
    // Noted slope here: k4
    State k4 = f(t + dt, y + (k3 * dt));

    // y_new = y + (dt/6) * (k1 + 2*k2 + 2_k3 + k4)
    // Average slope
    State slope_sum = k1 + (k2 * 2.0);
    slope_sum += k3 * 2.0;
    slope_sum += k4;

    return y + (slope_sum * (dt / 6.0));
}

} // namespace icarus