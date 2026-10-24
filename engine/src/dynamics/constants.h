#pragma once

namespace icarus {
// Physics Constants
constexpr double GRAVITY = 9.81;      // m/s^2
constexpr double MASS_DRY = 1000.0;   // kg (rocket dry mass)
constexpr double MAX_THRUST = 25000.0;  // N (for comfortable hover margin)
constexpr double MAX_FLOW_RATE = 5.0;  // kg/s at full throttle

// Controller Constants
constexpr double MIN_THROTTLE = 0.0;
constexpr double MAX_THROTTLE = 1.0;
}  // namespace icarus