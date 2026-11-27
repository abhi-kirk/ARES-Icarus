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

// How far the PID may tilt the thrust off vertical when the pointing cone
// is wider than this. A 90° cone lets a saturated side loop pitch the body
// onto its side once the up-force falls below the ceiling.
constexpr double PID_DIVERT_TILT_DEG = 35.0;

// Slice of the ceiling kept for the divert while the up-force is sitting
// on the limit. Small enough that a stop which needs the ceiling still
// has almost all of it.
constexpr double PID_DIVERT_RESERVE = 0.15;

// One engine. The PID writes the up-force and the side-force separately.
// Keep the up-force, fund a small divert even when that up-force fills the
// ceiling, and refuse a side command that would tilt past the divert angle
// (or the pointing cone, when the cone is tighter). MPC and SCvx already
// plan one vector, so they do not come through here.
inline void prefer_vertical_thrust(
    double& Fx,
    double& Fy,
    double& Fz,
    double thrust_max,
    double pointing_cone_deg
) {
    if (Fz < 0.0) Fz = 0.0;
    const double max_n = std::max(0.0, thrust_max);
    if (Fz > max_n) Fz = max_n;

    const double sideways = std::hypot(Fx, Fy);
    if (sideways <= 1e-12 || max_n <= 1e-12) {
        Fx = 0.0;
        Fy = 0.0;
        return;
    }

    double tilt_deg = PID_DIVERT_TILT_DEG;
    const double cone_deg = std::clamp(pointing_cone_deg, 0.0, 90.0);
    if (cone_deg < 89.999 && cone_deg < tilt_deg) tilt_deg = cone_deg;
    const double tilt_rad = tilt_deg * (3.14159265358979323846 / 180.0);
    const double tan_tilt = std::tan(tilt_rad);

    // A tight pointing cone already decides the divert, and spending the
    // ceiling on it ends the burn before that divert is finished. Fund the
    // slice only when the cone is open.
    const double reserve = (cone_deg < 89.999) ? 0.0 : PID_DIVERT_RESERVE * max_n;
    const double funded = std::min(sideways, reserve);
    const double fz_room = std::sqrt(std::max(0.0, max_n * max_n - funded * funded));
    if (Fz > fz_room) Fz = fz_room;

    const double tilt_cap = Fz * tan_tilt;
    const double circle = std::sqrt(std::max(0.0, max_n * max_n - Fz * Fz));
    double allowed = sideways;
    if (tilt_cap < allowed) allowed = tilt_cap;
    if (circle < allowed) allowed = circle;

    const double scale = allowed / sideways;
    Fx *= scale;
    Fy *= scale;
}

}  // namespace icarus
