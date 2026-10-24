#include "environment/landing_pad.h"

namespace icarus {

LandingPad::LandingPad(const SimConfig& config) {
    pos_x_init_ = config.pad_initial_x;
    pos_y_init_ = config.pad_initial_y;
    vel_x_ = config.pad_drift_vx;
    vel_y_ = config.pad_drift_vy;
}

Eigen::Vector2d LandingPad::pad_position(double t) const {
    Eigen::Vector2d pad_pos;

    double pos_x = pos_x_init_ + vel_x_ * t;
    double pos_y = pos_y_init_ + vel_y_ * t;

    pad_pos << pos_x, pos_y;
    
    return pad_pos;
}

Eigen::Vector2d LandingPad::pad_velocity() const {
    Eigen::Vector2d vel;
    vel << vel_x_, vel_y_;
    return vel;
}

}  // namespace icarus