#pragma once

#include "environment/gravity.h"

namespace icarus {

// The landing laws read east, north, and up at the pad. A planet-frame
// state uses +z through the pad, so the pad is at (0, 0, R) and a vehicle
// just above it has altitude z - R. This is the handoff into PID, MPC, and SCvx.
struct PadState {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
};

inline PadState local_state(
    double x,
    double y,
    double z,
    double vx,
    double vy,
    double vz
) {
    PadState pad;
    pad.x = x;
    pad.y = y;
    pad.z = z - EARTH_RADIUS_M;
    pad.vx = vx;
    pad.vy = vy;
    pad.vz = vz;
    return pad;
}

}  // namespace icarus
