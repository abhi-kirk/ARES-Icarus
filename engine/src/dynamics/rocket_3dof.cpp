#include "dynamics/rocket_3dof.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <cmath>
#include <Eigen/Dense>

using std::abs;

namespace icarus {

State rocket_dynamics_3dof(double t, const State& state, const ControlInput& control, const Disturbance& disturbance) {
    (void)t;

    const double wind_x = disturbance[0];
    const double wind_y = disturbance[1];
    const double wind_z = disturbance[2];

    const double vx = state[VEL_X] - wind_x;
    const double vy = state[VEL_Y] - wind_y;
    const double vz = state[VEL_Z] - wind_z;

    const double thrust_x = control[0];
    const double thrust_y = control[1];
    const double thrust_z = control[2];

    const double fuel = state[FUEL];
    const double m    = MASS_DRY + fuel;

    double m_dot = 0.0;
    if (fuel > 0.0) {
        double thrust_magnitude = control.norm();
        m_dot = -(thrust_magnitude / MAX_THRUST) * MAX_FLOW_RATE;
    }

    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    const double drag_x = -0.5 * vx * speed;
    const double drag_y = -0.5 * vy * speed;
    const double drag_z = -0.5 * vz * speed;

    const double net_force_x = thrust_x + drag_x;
    const double net_force_y = thrust_y + drag_y;
    const double net_force_z = thrust_z - (m * GRAVITY) + drag_z;

    const double accel_x = net_force_x / m;
    const double accel_y = net_force_y / m;
    const double accel_z = net_force_z / m;

    State state_dot(STATE_SIZE);
    state_dot << state[VEL_X], state[VEL_Y], state[VEL_Z], accel_x, accel_y, accel_z, m_dot;
    return state_dot;
}

}  // namespace icarus