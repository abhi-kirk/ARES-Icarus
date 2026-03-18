#pragma once

#include "config/config.h"
#include <Eigen/Dense>

namespace icarus {

class LandingPad {
public:
    explicit LandingPad(const SimConfig& config);

    // Returns landing pad position [lx, ly] at time t
    Eigen::Vector2d pad_position(double t) const;

    // Returns constant pad drift velocity [vx, vy]
    Eigen::Vector2d pad_velocity() const;

private:
    double pos_x_init_;
    double pos_y_init_;
    double vel_x_;
    double vel_y_;
};

}  // namespace icarus