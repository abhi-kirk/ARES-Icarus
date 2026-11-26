#pragma once

#include <algorithm>
#include <cmath>

namespace icarus {

// Turn a desired body-frame force into thrust and gimbal. The body is
// already being pointed along the inertial force, so what remains here is
// the angle it has not reached yet. The jet torque is applied separately.
inline void allocate_gimbal(
    double Fx_des,
    double Fy_des,
    double Fz_des,
    double gimbal_max_rad,
    double& T,
    double& delta_x,
    double& delta_y
) {
    T = std::sqrt(Fx_des * Fx_des + Fy_des * Fy_des + Fz_des * Fz_des);
    if (T < 1e-9) {
        delta_x = 0.0;
        delta_y = 0.0;
        return;
    }

    // Fx = T sin(dy), Fy = -T sin(dx), Fz = T cos(dx)cos(dy) when upright.
    double dx = std::atan2(-Fy_des, Fz_des);
    double dy = std::atan2(Fx_des, Fz_des);

    const double max_a = std::max(0.0, gimbal_max_rad);
    delta_x = std::clamp(dx, -max_a, max_a);
    delta_y = std::clamp(dy, -max_a, max_a);
}

}  // namespace icarus
