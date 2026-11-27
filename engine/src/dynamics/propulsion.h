#pragma once

#include "dynamics/constants.h"

namespace icarus {

// Kilograms per second leaving the tank at this thrust.
// 25,000 N at 5,000 m/s of exhaust is the old 5 kg/s full-throttle flow.
inline double fuel_flow_kg_s(double thrust_n, double exhaust_speed_m_s) {
    if (thrust_n <= 0.0 || exhaust_speed_m_s <= 1.0) {
        return 0.0;
    }
    return thrust_n / exhaust_speed_m_s;
}

}  // namespace icarus
