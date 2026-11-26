#include "dynamics/rocket_3dof.h"
#include "dynamics/air.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

using std::abs;

namespace icarus {

State rocket_dynamics_3dof(
    double t,
    const State& state,
    const ControlInput& control,
    const Disturbance& disturbance,
    double drag_scale,
    double gravity
) {
    (void)t;

    const double wind_x = disturbance[0];
    const double wind_y = disturbance[1];
    const double wind_z = disturbance[2];

    const double vx = state[VEL_X] - wind_x;
    const double vy = state[VEL_Y] - wind_y;
    const double vz = state[VEL_Z] - wind_z;

    const double fuel = state[FUEL];
    // An empty tank produces no thrust. The command can still ask for it.
    const double thrust_x = (fuel > 0.0) ? control[0] : 0.0;
    const double thrust_y = (fuel > 0.0) ? control[1] : 0.0;
    const double thrust_z = (fuel > 0.0) ? control[2] : 0.0;

    const double m = MASS_DRY + std::max(0.0, fuel);

    double m_dot = 0.0;
    if (fuel > 0.0) {
        double thrust_magnitude = std::sqrt(
            thrust_x * thrust_x + thrust_y * thrust_y + thrust_z * thrust_z
        );
        m_dot = -(thrust_magnitude / MAX_THRUST) * MAX_FLOW_RATE;
    }

    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
    const double density = air_density_ratio(state[POS_Z], gravity);
    const double drag_coeff = 0.5 * std::max(0.0, drag_scale) * density;

    const double drag_x = -drag_coeff * vx * speed;
    const double drag_y = -drag_coeff * vy * speed;
    const double drag_z = -drag_coeff * vz * speed;

    const double net_force_x = thrust_x + drag_x;
    const double net_force_y = thrust_y + drag_y;
    const double net_force_z = thrust_z - (m * gravity) + drag_z;

    const double accel_x = net_force_x / m;
    const double accel_y = net_force_y / m;
    const double accel_z = net_force_z / m;

    // Quaternion and rate derivatives stay zero: this plant has no rotation.
    // The caller initializes the quaternion to identity.
    State state_dot(STATE_SIZE);
    state_dot << state[VEL_X], state[VEL_Y], state[VEL_Z],
                 accel_x, accel_y, accel_z,
                 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 m_dot;
    return state_dot;
}

}  // namespace icarus