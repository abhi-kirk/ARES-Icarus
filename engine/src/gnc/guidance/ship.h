#pragma once

namespace icarus {

// One ship, just after the booster is gone. Round numbers, not a CAD model.
// Dry mass stays 1,000 kg. 5,000 kg of propellant at 5,000 m/s of exhaust
// is about 7.8 km/s of rocket delta-v above a 400 kg landing reserve
// (5,000 * ln(6000 / 1400)). The burn is 100,000 N, so full thrust is 20 kg/s.
constexpr double SHIP_THRUST_N = 100000.0;
constexpr double SHIP_EXHAUST_M_S = 5000.0;
constexpr double SHIP_FUEL_KG = 5000.0;
constexpr double SHIP_RESERVE_KG = 400.0;
constexpr double SHIP_START_ALTITUDE_M = 100000.0;
constexpr double SHIP_START_SPEED_M_S = 2000.0;
constexpr double SHIP_START_GAMMA_DEG = 23.5;
constexpr double SHIP_KICK_DEG = 6.0;
constexpr double SHIP_KICK_S = 10.0;
// Pascals. The ascent throttle drops while dynamic pressure is above this.
constexpr double SHIP_MAX_Q_PA = 35000.0;
// Integral of sqrt(rho) * v^3. The 1 km/s coast stays under this.
// 7.8 km/s in thick air does not.
constexpr double SHIP_SHIELD = 1.0e11;

}  // namespace icarus
