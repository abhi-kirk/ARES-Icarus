#pragma once

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Sea-level density already lives inside drag_scale. This ratio is 1 on the
// pad and falls off with height. 8.5 km is the scale height of that falloff.
constexpr double AIR_SCALE_HEIGHT_M = 8500.0;

// Earth keeps the exponential atmosphere. Lunar gravity is airless for the
// whole flight, so a Moon case stays vacuum even if drag_scale is left on.
inline double air_density_ratio(double altitude_m, double gravity) {
    const bool lunar = std::abs(gravity - GRAVITY_MOON) < 0.05;
    if (lunar) {
        return 0.0;
    }
    const double height = std::max(0.0, altitude_m);
    return std::exp(-height / AIR_SCALE_HEIGHT_M);
}

}  // namespace icarus
