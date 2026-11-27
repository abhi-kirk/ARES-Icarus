#pragma once

#include "dynamics/air.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Drag force in newtons. Attitude does not enter: the same law the 6-DOF
// plant and the entry coast both used when drag lived inside each file.
// drag_scale 1 is the historical sea-level force. Lunar gravity is vacuum.
inline void aero_force(
    double vx,
    double vy,
    double vz,
    double wind_x,
    double wind_y,
    double wind_z,
    double altitude_m,
    double drag_scale,
    double gravity,
    double& fx,
    double& fy,
    double& fz
) {
    const double vx_rel = vx - wind_x;
    const double vy_rel = vy - wind_y;
    const double vz_rel = vz - wind_z;
    const double speed = std::sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
    const double density = air_density_ratio(altitude_m, gravity);
    const double drag_coeff = 0.5 * std::max(0.0, drag_scale) * density;
    fx = -drag_coeff * vx_rel * speed;
    fy = -drag_coeff * vy_rel * speed;
    fz = -drag_coeff * vz_rel * speed;
}

}  // namespace icarus
