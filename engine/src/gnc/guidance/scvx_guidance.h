#pragma once

#include <vector>

#include <Eigen/Dense>

#include "gnc/guidance/ltv_mpc.h"

namespace icarus {

// Fuel-optimal translation plan.
//
// Each pass freezes the rocket's mass, solves one convex fuel problem, then
// updates the mass from the thrust it just chose. A trust region keeps the
// new thrust near the thrust the mass was frozen at. With mass frozen, the
// motion is linear, so this slice does not add a fake extra acceleration.
//
// The thrust length limit is a ball. A slack variable sits between the
// floor and the ceiling, and the thrust arrow is no longer than that slack.
// The 15 degree tilt stays four straight cuts around vertical. QOCO solves
// this cone problem. The tracking planner still uses OSQP.
//
// Several flight durations are tried. Shorter flights use less fuel on this
// plant, because the engine cannot shut off. The shortest duration that
// still soft-lands is kept.
class ScvxGuidance {
public:
    void configure(
        double thrust_min,
        double thrust_max,
        double pointing_cone_deg,
        double gimbal_max_deg,
        double glide_cone_deg,
        double drag_scale
    );

    // Rebuild the plan from the state at time t. pad and wind are held
    // still for the whole plan.
    MpcSolveResult solve(
        double t,
        const Eigen::Vector3d& pos,
        const Eigen::Vector3d& vel,
        double mass,
        double pad_x,
        double pad_y,
        double wind_x,
        double wind_y
    );

    // Inertial force (N) the plan wants at absolute time t_query.
    Eigen::Vector3d force_at(double t_query) const;

    const MpcSolveResult& plan() const { return plan_; }

private:
    double thrust_min_ = 0.0;
    double thrust_max_ = 25000.0;
    double pointing_cone_deg_ = 90.0;
    double glide_cone_deg_ = 90.0;
    double drag_scale_ = 1.0;

    double t0_ = 0.0;
    double dt_ = 0.5;
    double sink_fz_ = 0.0;
    std::vector<Eigen::Vector3d> force_;
    MpcSolveResult plan_;
};

}  // namespace icarus
