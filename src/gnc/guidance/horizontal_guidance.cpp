#include "gnc/guidance/horizontal_guidance.h"
#include "dynamics/state_indices.h"

#include <algorithm>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Vector2d;

namespace icarus {

static double rate_limit(double current, double target, double max_rate, double dt) {
    if (max_rate <= 0.0 || dt <= 0.0) return target;
    const double delta = target - current;
    const double max_delta = max_rate * dt;
    return current + std::clamp(delta, -max_delta, max_delta);
}

HorizontalGuidance::HorizontalGuidance(const Config& config)
    : pos_ctrl_x_(config.simulation.timestep_outer_h,
                  PIDSpec{config.pid.kp_h_pos, config.pid.ki_h_pos, config.pid.kd_h_pos, -10.0, 10.0}),
      pos_ctrl_y_(config.simulation.timestep_outer_h,
                  PIDSpec{config.pid.kp_h_pos, config.pid.ki_h_pos, config.pid.kd_h_pos, -10.0, 10.0})
{
    max_ref_accel_ = config.simulation.max_ref_accel_h;
    dt_inner_      = config.simulation.timestep_inner_h;

    double ratio        = config.simulation.timestep_outer_h / config.simulation.timestep_inner_h;
    outer_loop_divider_ = std::max(1, static_cast<int>(std::lround(ratio)));
}

void HorizontalGuidance::initialize(const State& state, const Vector2d& pad_position,
                                     const Vector2d& pad_velocity) {
    ref_vel_target_ = {
        pos_ctrl_x_.compute(state[POS_X], pad_position[0]) + pad_velocity[0],
        pos_ctrl_y_.compute(state[POS_Y], pad_position[1]) + pad_velocity[1]
    };
    ref_vel_cmd_ = ref_vel_target_;
}

Vector2d HorizontalGuidance::compute(const State& state, const Vector2d& pad_position,
                                      const Vector2d& pad_velocity) {
    if (iteration_ % outer_loop_divider_ == 0) {
        // PID drives position error to zero; pad_velocity feedforward lets the
        // controller track a moving target without steady-state lag.
        ref_vel_target_ = {
            pos_ctrl_x_.compute(state[POS_X], pad_position[0]) + pad_velocity[0],
            pos_ctrl_y_.compute(state[POS_Y], pad_position[1]) + pad_velocity[1]
        };
    }
    ref_vel_cmd_ = {
        rate_limit(ref_vel_cmd_[0], ref_vel_target_[0], max_ref_accel_, dt_inner_), 
        rate_limit(ref_vel_cmd_[1], ref_vel_target_[1], max_ref_accel_, dt_inner_)
    };
    ++iteration_;
    return ref_vel_cmd_;
}

}  // namespace icarus