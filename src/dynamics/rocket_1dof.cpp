#include "dynamics/rocket_1dof.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <cmath>

using std::abs;

namespace icarus {

State rocket_dynamics(double t, const State& state, double throttle) {
    (void)t;

    const double v    = state[VEL];
    const double fuel = state[FUEL];
    const double m    = MASS_DRY + fuel;

    double thrust = 0.0;
    double m_dot  = 0.0;
    if (fuel > 0.0) {
        thrust = throttle * MAX_THRUST;
        m_dot  = -throttle * MAX_FLOW_RATE;
    }

    // Quadratic drag: opposes velocity (positive when falling, negative when rising)
    const double drag      = -0.5 * v * abs(v);
    const double net_force = thrust - (m * GRAVITY) - drag;
    const double accel     = net_force / m;

    State state_dot(3);
    state_dot << v, accel, m_dot;
    return state_dot;
}

}  // namespace icarus
