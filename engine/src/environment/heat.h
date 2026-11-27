#pragma once

#include "dynamics/air.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Heating rate along the flight. sqrt(rho) * v^3, the usual convective
// estimate. No chemistry and no plasma. The integral is compared with the
// shield capacity on the case. A capacity of 0 means the flight does not
// check heat, which is every hopper.
inline double heat_rate(double altitude_m, double speed_m_s, double gravity) {
    const double rho = air_density(altitude_m, gravity);
    if (rho <= 0.0) {
        return 0.0;
    }
    const double speed = std::max(0.0, speed_m_s);
    return std::sqrt(rho) * speed * speed * speed;
}

}  // namespace icarus
