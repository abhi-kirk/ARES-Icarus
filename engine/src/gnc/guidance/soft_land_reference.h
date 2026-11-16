#pragma once

#include <vector>

#include <Eigen/Dense>

namespace icarus {

// One knot on the soft-landing reference the LTV-MPC tracks.
struct ReferenceKnot {
    double t = 0.0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();  // inertial, pad-relative later
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d force = Eigen::Vector3d::Zero();  // Fx, Fy, Fz ask
};

// Builds a constant-accel soft-landing profile from the start state to the
// pad. Thrust is clamped to [thrust_min, thrust_max]. The profile is open
// loop and is only a reference for the QP — the plant still flies closed loop.
class SoftLandReference {
public:
    void build(
        const Eigen::Vector3d& pos0,
        const Eigen::Vector3d& vel0,
        double mass,
        double thrust_min,
        double thrust_max,
        double dt,
        double pad_x,
        double pad_y
    );

    // Sample the reference at absolute time t (clamped to the profile).
    ReferenceKnot at(double t) const;

    double duration() const { return duration_; }
    const std::vector<ReferenceKnot>& knots() const { return knots_; }

private:
    std::vector<ReferenceKnot> knots_;
    double duration_ = 0.0;
    double dt_ = 0.1;
};

}  // namespace icarus
