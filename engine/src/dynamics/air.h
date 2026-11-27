#pragma once

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Sea-level density already lives inside drag_scale. This ratio is 1 on the
// pad and falls off with height. 8.5 km is the scale height of that falloff.
constexpr double AIR_SCALE_HEIGHT_M = 8500.0;

// Standard sea-level density, kg/m³. The drag law uses the ratio above.
// The readout shows this density, so the pad reads 1.225 kg/m³.
constexpr double SEA_LEVEL_DENSITY = 1.225;

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

// Air density in kg/m³. Zero for the whole Moon flight.
inline double air_density(double altitude_m, double gravity) {
    return SEA_LEVEL_DENSITY * air_density_ratio(altitude_m, gravity);
}

}  // namespace icarus
