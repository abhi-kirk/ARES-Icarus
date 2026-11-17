#pragma once

#include <vector>

#include <Eigen/Dense>

namespace icarus {

// One knot on the soft-landing reference the LTV-MPC tracks.
struct ReferenceKnot {
    double t = 0.0;  // absolute flight time
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d force = Eigen::Vector3d::Zero();  // Fx, Fy, Fz ask
};

// Builds a constant-accel soft-landing profile from the current state to the
// pad. Thrust is clamped to [thrust_min, thrust_max]. The Simulator rebuilds
// this every MPC tick so wind and tracking error cannot stale the profile.
class SoftLandReference {
public:
    void build(
        double t0,
        const Eigen::Vector3d& pos0,
        const Eigen::Vector3d& vel0,
        double mass,
        double thrust_min,
        double thrust_max,
        double dt,
        double pad_x,
        double pad_y
    );

    // Sample at absolute flight time t (clamped to the profile).
    ReferenceKnot at(double t) const;

    double t0() const { return t0_; }
    double duration() const { return duration_; }
    double mass_used() const { return mass_used_; }
    const std::vector<ReferenceKnot>& knots() const { return knots_; }

private:
    std::vector<ReferenceKnot> knots_;
    double t0_ = 0.0;
    double duration_ = 0.0;
    double dt_ = 0.1;
    double mass_used_ = 0.0;
};

}  // namespace icarus
