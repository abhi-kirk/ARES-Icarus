#pragma once

#include <vector>

#include <Eigen/Dense>

#include "gnc/guidance/soft_land_reference.h"

namespace icarus {

struct MpcSolveResult {
    // First force to apply now (N).
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    // Predicted inertial positions over the horizon (including step 0 = now).
    std::vector<Eigen::Vector3d> plan_pos;
    bool ok = false;
};

// Condensed LTV-MPC on the translation plant. Linearizes a double integrator
// with gravity, tracks SoftLandReference, and solves a QP with OSQP.
class LtvMpc {
public:
    void configure(
        int horizon,
        double dt,
        double w_pos,
        double w_vel,
        double w_u,
        double w_terminal,
        double thrust_min,
        double thrust_max,
        double pointing_cone_deg
    );

    void set_reference(const SoftLandReference* reference);

    // state_pos / state_vel are inertial. pad is the current pad location.
    // mass is dry + fuel. t is absolute flight time (matches the reference).
    MpcSolveResult solve(
        double t,
        const Eigen::Vector3d& state_pos,
        const Eigen::Vector3d& state_vel,
        double mass,
        double pad_x,
        double pad_y
    );

private:
    int horizon_ = 25;
    double dt_ = 0.1;
    double w_pos_ = 4.0;
    double w_vel_ = 1.0;
    double w_u_ = 1.0e-8;
    double w_terminal_ = 20.0;
    double thrust_min_ = 0.0;
    double thrust_max_ = 25000.0;
    double pointing_cone_deg_ = 90.0;
    const SoftLandReference* reference_ = nullptr;
};

}  // namespace icarus
