#pragma once

#include <algorithm>
#include <cmath>

namespace icarus {

// Clamp a 3DOF thrust command to a pointing cone from vertical and a
// magnitude ring [thrust_min, thrust_max]. Order: keep Fz >= 0, cut
// horizontal to the cone, then scale the whole vector onto the ring.
// Direction is preserved when the magnitude is scaled, so the cone still
// holds after that step.
inline void apply_thrust_limits(
    double& Fx,
    double& Fy,
    double& Fz,
    double thrust_min,
    double thrust_max,
    double pointing_cone_deg
) {
    if (Fz < 0.0) Fz = 0.0;

    const double min_n = std::max(0.0, thrust_min);
    const double max_n = std::max(min_n, thrust_max);

    // Cap tilt from vertical. At 90 deg the tan grows without bound, so
    // treat that as "no pointing limit" and skip the cut.
    const double cone_deg = std::clamp(pointing_cone_deg, 0.0, 90.0);
    if (cone_deg < 89.999) {
        const double cone_rad = cone_deg * (3.14159265358979323846 / 180.0);
        const double tan_cone = std::tan(cone_rad);
        const double fh = std::hypot(Fx, Fy);
        const double max_fh = Fz * tan_cone;
        if (fh > max_fh && fh > 1e-12) {
            const double scale = max_fh / fh;
            Fx *= scale;
            Fy *= scale;
        }
    }

    const double mag = std::hypot(Fx, std::hypot(Fy, Fz));
    if (mag > max_n && mag > 1e-12) {
        const double scale = max_n / mag;
        Fx *= scale;
        Fy *= scale;
        Fz *= scale;
        return;
    }
    if (mag < min_n && mag > 1e-12) {
        const double scale = min_n / mag;
        Fx *= scale;
        Fy *= scale;
        Fz *= scale;
        return;
    }
    if (mag <= 1e-12 && min_n > 0.0) {
        // No command, but the engine cannot sit below the floor: fire up.
        Fx = 0.0;
        Fy = 0.0;
        Fz = min_n;
    }
}

}  // namespace icarus
