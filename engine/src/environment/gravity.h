#pragma once

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// Mean Earth radius, meters. The pad sits on this sphere.
constexpr double EARTH_RADIUS_M = 6371000.0;

// True for the lunar surface gravity the cases already use.
inline bool lunar_gravity(double gravity) {
    return std::abs(gravity - GRAVITY_MOON) < 0.05;
}

// g0 * (R / r)^2. r is the distance from Earth's center.
inline double gravity_at_radius(double g0, double radius_m) {
    const double radius = std::max(radius_m, 1.0);
    const double ratio = EARTH_RADIUS_M / radius;
    return g0 * ratio * ratio;
}

// Altitude above the surface. In the pad frame z is already that altitude.
// In the planet frame position is meters from Earth's center.
inline double altitude_m(double x, double y, double z, bool planet_centered) {
    if (!planet_centered) {
        return z;
    }
    const double radius = std::sqrt(x * x + y * y + z * z);
    return radius - EARTH_RADIUS_M;
}

// Acceleration from gravity, m/s².
// Pad frame: x east, y north, z up. The pull stays straight down so the
// landing laws still read z as altitude, and the size is g0*(R/(R+z))^2.
// Planet frame: position is from Earth's center and the pull points there.
// The Moon stays a constant 1.62 m/s² straight down in the pad frame.
inline void gravity_accel(
    double g0,
    double x,
    double y,
    double z,
    bool planet_centered,
    double& ax,
    double& ay,
    double& az
) {
    if (lunar_gravity(g0) || !planet_centered) {
        double g = g0;
        if (!lunar_gravity(g0)) {
            g = gravity_at_radius(g0, EARTH_RADIUS_M + z);
        }
        ax = 0.0;
        ay = 0.0;
        az = -g;
        return;
    }

    const double radius_sq = x * x + y * y + z * z;
    const double radius = std::sqrt(std::max(radius_sq, 1.0));
    const double g = gravity_at_radius(g0, radius);
    ax = -g * x / radius;
    ay = -g * y / radius;
    az = -g * z / radius;
}

}  // namespace icarus
