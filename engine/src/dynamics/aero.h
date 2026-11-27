#pragma once

#include "dynamics/air.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Dynamic pressure, pascals. 0.5 * rho * v^2.
inline double dynamic_pressure(double altitude_m, double speed_m_s, double gravity) {
    const double speed = std::max(0.0, speed_m_s);
    return 0.5 * air_density(altitude_m, gravity) * speed * speed;
}

// Throttle bucket. Above q_limit the thrust scales down so dynamic
// pressure stays at the limit. Below the limit the command is unchanged.
inline double cap_thrust_for_q(double thrust_n, double q_pa, double q_limit_pa) {
    if (thrust_n <= 0.0 || q_limit_pa <= 0.0 || q_pa <= q_limit_pa) {
        return thrust_n;
    }
    return thrust_n * (q_limit_pa / q_pa);
}

// Drag force in newtons. With use_aero false, attitude does not enter:
// the same law the hopper and the entry coast already fly.
// drag_scale 1 is the historical sea-level force. Lunar gravity is vacuum.
// With use_aero true, drag grows with angle of attack and lift appears
// toward the nose. Nose-on to the velocity matches the old drag.
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
    double& fz,
    bool use_aero = false,
    double nose_x = 0.0,
    double nose_y = 0.0,
    double nose_z = 1.0
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
    if (!use_aero || speed < 1e-6 || density <= 0.0) {
        return;
    }

    // Old force magnitude is q * (drag_scale / sea-level density).
    // That product is Cd*A at zero angle of attack.
    const double q = 0.5 * SEA_LEVEL_DENSITY * density * speed * speed;
    const double cd_area = std::max(0.0, drag_scale) / SEA_LEVEL_DENSITY;
    const double vhx = vx_rel / speed;
    const double vhy = vy_rel / speed;
    const double vhz = vz_rel / speed;
    const double nose_dot_v = std::clamp(
        nose_x * vhx + nose_y * vhy + nose_z * vhz,
        -1.0,
        1.0
    );
    const double alpha = std::acos(nose_dot_v);

    const double drag = q * cd_area * (1.0 + 2.0 * alpha * alpha);
    fx = -drag * vhx;
    fy = -drag * vhy;
    fz = -drag * vhz;

    // Lift is along the part of the nose that is not the velocity.
    const double lx = nose_x - nose_dot_v * vhx;
    const double ly = nose_y - nose_dot_v * vhy;
    const double lz = nose_z - nose_dot_v * vhz;
    const double lift_len = std::sqrt(lx * lx + ly * ly + lz * lz);
    if (lift_len < 1e-8) {
        return;
    }
    const double lift = q * cd_area * 2.0 * std::sin(alpha);
    fx += lift * lx / lift_len;
    fy += lift * ly / lift_len;
    fz += lift * lz / lift_len;
}

}  // namespace icarus
