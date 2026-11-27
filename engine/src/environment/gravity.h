#pragma once

namespace icarus {

// Acceleration from gravity, m/s², in the pad frame. Up is +z.
// This commit is still a constant pull straight down. A later commit
// points it at the center of a round Earth.
inline void gravity_accel(double gravity, double& ax, double& ay, double& az) {
    ax = 0.0;
    ay = 0.0;
    az = -gravity;
}

}  // namespace icarus
