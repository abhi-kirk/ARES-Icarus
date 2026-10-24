#pragma once

#include "config/config.h"
#include "gnc/control/controller.h"
#include "integrator.h"

#include <Eigen/Dense>

namespace icarus {

class HorizontalGuidance {
public:
    explicit HorizontalGuidance(const Config& config);

    void initialize(const State& state, const Eigen::Vector2d& pad_position,
                    const Eigen::Vector2d& pad_velocity);

    Eigen::Vector2d compute(const State& state, const Eigen::Vector2d& pad_position,
                            const Eigen::Vector2d& pad_velocity);

    Eigen::Vector2d ref_vel_target() const { return ref_vel_target_; }

private:
    Controller pos_ctrl_x_;
    Controller pos_ctrl_y_;

    Eigen::Vector2d ref_vel_target_ = {0.0, 0.0};
    Eigen::Vector2d ref_vel_cmd_ = {0.0, 0.0};

    double max_ref_accel_;
    double dt_inner_;

    int outer_loop_divider_;
    int iteration_ = 0;
};

}  // namespace icarus