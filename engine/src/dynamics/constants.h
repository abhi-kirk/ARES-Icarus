#pragma once

namespace icarus {
// Physics Constants
constexpr double GRAVITY_EARTH = 9.81;  // m/s^2
constexpr double GRAVITY_MOON = 1.62;   // m/s^2
constexpr double GRAVITY = GRAVITY_EARTH;
constexpr double MASS_DRY = 1000.0;   // kg (rocket dry mass)
constexpr double MAX_THRUST = 25000.0;  // N (for comfortable hover margin)
constexpr double MAX_FLOW_RATE = 5.0;  // kg/s at full throttle
// Exhaust speed that makes full thrust burn MAX_FLOW_RATE. 25000 / 5 = 5000 m/s.
constexpr double EXHAUST_SPEED = MAX_THRUST / MAX_FLOW_RATE;

// Rigid-body defaults for the 6-DOF plant. Ixx = Iyy (slender stack).
// Lever is CG to gimbal along -body z. Cases may override these.
constexpr double IXX_DEFAULT = 2000.0;   // kg·m^2
constexpr double IYY_DEFAULT = 2000.0;   // kg·m^2
constexpr double IZZ_DEFAULT = 400.0;    // kg·m^2
constexpr double GIMBAL_LEVER_DEFAULT = 8.0;  // m
constexpr double GIMBAL_MAX_DEG_DEFAULT = 15.0;

// Controller Constants
constexpr double MIN_THROTTLE = 0.0;
constexpr double MAX_THROTTLE = 1.0;
}  // namespace icarus
