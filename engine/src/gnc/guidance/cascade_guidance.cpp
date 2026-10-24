#include "gnc/guidance/cascade_guidance.h"
#include "dynamics/state_indices.h"

#include <algorithm>
#include <cmath>

namespace icarus {

static double rate_limit(double current, double target, double max_rate, double dt) {
    if (max_rate <= 0.0 || dt <= 0.0) return target;
    const double delta = target - current;
    const double max_delta = max_rate * dt;
    return current + std::clamp(delta, -max_delta, max_delta);
}

CascadeGuidance::CascadeGuidance(const Config& config)
    : pos_ctrl_(config.simulation.timestep_outer,
                PIDSpec{config.pid.kp_v_pos, config.pid.ki_v_pos, config.pid.kd_v_pos, -20.0, 0.0})
{
    // pos_ctrl_ must be initialized above (in the initializer list) because
    // Controller has no default constructor — it needs dt and gains at creation.
    // Everything else can be set normally here.

    max_ref_accel_ = config.simulation.max_ref_accel_v;
    dt_inner_      = config.simulation.timestep_inner;

    // How many inner-loop ticks fit in one outer-loop tick?
    // e.g. timestep_outer=0.1, timestep_inner=0.02 -> divider=5
    // outer loop fires on ticks: 0, 5, 10, 15, ...
    double ratio        = config.simulation.timestep_outer / config.simulation.timestep_inner;
    outer_loop_divider_ = std::max(1, static_cast<int>(std::lround(ratio)));
}

void CascadeGuidance::initialize(const State& state) {
    ref_vel_target_ = pos_ctrl_.compute(state[POS_Z], 0.0);
    ref_vel_cmd_    = ref_vel_target_;
}

double CascadeGuidance::compute(const State& state) {
    if (iteration_ % outer_loop_divider_ == 0) {
        ref_vel_target_ = pos_ctrl_.compute(state[POS_Z], 0.0);
    }
    ref_vel_cmd_ = rate_limit(ref_vel_cmd_, ref_vel_target_, max_ref_accel_, dt_inner_);

    // Enforce a minimum descent rate so the rocket doesn't asymptotically hover
    // to the ground. A P-only outer loop drives ref_vel -> 0 as altitude -> 0,
    // which causes the sim to run indefinitely. This floor ensures the rocket
    // always descends at least MIN_DESCENT_VEL m/s and reaches the ground.
    static constexpr double MIN_DESCENT_VEL = -0.3;  // m/s
    ref_vel_cmd_ = std::min(ref_vel_cmd_, MIN_DESCENT_VEL);

    ++iteration_;
    return ref_vel_cmd_;
}

}  // namespace icarus
