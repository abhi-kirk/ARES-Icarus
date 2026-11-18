#pragma once

#include "dynamics/constants.h"

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
// with gravity and a local drag/wind model, tracks SoftLandReference, and
// solves a QP with OSQP. Warm-starts from the previous solution (shifted one
// step). Glide-slope inequalities are soft (slack + penalty).
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
        double pointing_cone_deg,
        double glide_cone_deg,
        double drag_scale,
        double gravity = GRAVITY
    );

    void set_reference(const SoftLandReference* reference);

    // state_pos / state_vel are inertial. pad is the current pad location.
    // mass is dry + fuel. wind is the current wind estimate (held over the
    // horizon). t is absolute flight time (matches the reference).
    MpcSolveResult solve(
        double t,
        const Eigen::Vector3d& state_pos,
        const Eigen::Vector3d& state_vel,
        double mass,
        double pad_x,
        double pad_y,
        double wind_x,
        double wind_y
    );

private:
    int horizon_ = 50;
    double dt_ = 0.1;
    double w_pos_ = 40.0;
    double w_vel_ = 50.0;
    double w_u_ = 1.0e-9;
    double w_terminal_ = 120.0;
    double thrust_min_ = 0.0;
    double thrust_max_ = 25000.0;
    double pointing_cone_deg_ = 90.0;
    double glide_cone_deg_ = 90.0;
    double drag_scale_ = 1.0;
    double gravity_ = GRAVITY;
    const SoftLandReference* reference_ = nullptr;

    // Previous control sequence for OSQP warm start (length 3 * horizon).
    Eigen::VectorXd last_U_;
    bool have_warm_ = false;
};

}  // namespace icarus
