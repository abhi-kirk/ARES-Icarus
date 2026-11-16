#include "gnc/guidance/horizontal_guidance.h"
#include "dynamics/state_indices.h"

#include <algorithm>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Vector2d;

namespace icarus {

namespace {

constexpr double PI = 3.14159265358979323846;

static double rate_limit(double current, double target, double max_rate, double dt) {
    if (max_rate <= 0.0 || dt <= 0.0) return target;
    const double delta = target - current;
    const double max_delta = max_rate * dt;
    return current + std::clamp(delta, -max_delta, max_delta);
}

// Soft approach-corridor pull. When range from the pad exceeds
// height * tan(cone), add an inward velocity demand proportional to the
// meters past the wall. At 90 deg the wall is ignored.
static Vector2d glide_pull_velocity(
    const State& state,
    const Vector2d& pad_position,
    double glide_cone_deg,
    double glide_pull
) {
    Vector2d pull = Vector2d::Zero();
    const double cone_deg = std::clamp(glide_cone_deg, 0.0, 90.0);
    if (cone_deg >= 89.999 || glide_pull <= 0.0) return pull;

    const double err_x = state[POS_X] - pad_position[0];
    const double err_y = state[POS_Y] - pad_position[1];
    const double range = std::hypot(err_x, err_y);
    if (range <= 1e-9) return pull;

    const double height = std::max(state[POS_Z], 1.0);
    const double wall = height * std::tan(cone_deg * (PI / 180.0));
    if (range <= wall) return pull;

    const double excess = range - wall;
    const double speed = glide_pull * excess;
    // Unit vector from pad to vehicle; pull is toward the pad.
    pull[0] = -speed * (err_x / range);
    pull[1] = -speed * (err_y / range);
    return pull;
}

}  // namespace

HorizontalGuidance::HorizontalGuidance(const Config& config)
    : pos_ctrl_x_(config.simulation.timestep_outer_h,
                  PIDSpec{config.pid.kp_h_pos, config.pid.ki_h_pos, config.pid.kd_h_pos, -10.0, 10.0}),
      pos_ctrl_y_(config.simulation.timestep_outer_h,
                  PIDSpec{config.pid.kp_h_pos, config.pid.ki_h_pos, config.pid.kd_h_pos, -10.0, 10.0})
{
    max_ref_accel_ = config.simulation.max_ref_accel_h;
    dt_inner_      = config.simulation.timestep_inner_h;
    glide_cone_deg_ = config.simulation.glide_cone_deg;
    glide_pull_     = config.simulation.glide_pull;

    double ratio        = config.simulation.timestep_outer_h / config.simulation.timestep_inner_h;
    outer_loop_divider_ = std::max(1, static_cast<int>(std::lround(ratio)));
}

void HorizontalGuidance::initialize(const State& state, const Vector2d& pad_position,
                                     const Vector2d& pad_velocity) {
    ref_vel_target_ = {
        pos_ctrl_x_.compute(state[POS_X], pad_position[0]) + pad_velocity[0],
        pos_ctrl_y_.compute(state[POS_Y], pad_position[1]) + pad_velocity[1]
    };
    // Corridor is layered on in compute(), not stored in the rate-limited state.
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

    // Rate-limit the PID path only. The glide corridor is added after that
    // so a vehicle outside the wall gets an immediate inward push the
    // accel limiter cannot erase.
    ref_vel_cmd_ = {
        rate_limit(ref_vel_cmd_[0], ref_vel_target_[0], max_ref_accel_, dt_inner_),
        rate_limit(ref_vel_cmd_[1], ref_vel_target_[1], max_ref_accel_, dt_inner_)
    };
    const Vector2d corridor = glide_pull_velocity(state, pad_position, glide_cone_deg_, glide_pull_);
    Vector2d commanded = {
        ref_vel_cmd_[0] + corridor[0],
        ref_vel_cmd_[1] + corridor[1]
    };

    ++iteration_;
    return commanded;
}

}  // namespace icarus
