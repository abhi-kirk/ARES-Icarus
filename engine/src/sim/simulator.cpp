#include "sim/simulator.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <algorithm>
#include <cmath>

namespace icarus {

void Simulator::run(
    State& state, 
    double t_start,
    const SimConfig& cfg,
    CascadeGuidance& guidance,
    HorizontalGuidance& guidance_h,
    Controller& vel_ctrl,
    Controller& vel_ctrl_h_x,
    Controller& vel_ctrl_h_y,
    const DynamicsFunction& dynamics,
    const WindModel& wind_model,
    const LandingPad& landing_pad,
    Logger& logger
) {
    double t  = t_start;
    double dt_inner_v = cfg.timestep_inner;
    double dt_inner_h = cfg.timestep_inner_h;

    // How many sim steps per horizontal inner tick
    double ratio = dt_inner_h / dt_inner_v;
    int inner_loop_divider = std::max(1, static_cast<int>(std::lround(ratio)));
    int sim_counter = 0;

    Eigen::Vector2d ref_vel_xy = Eigen::Vector2d::Zero();
    double Fx = vel_ctrl_h_x.compute(state[VEL_X], 0.0);
    double Fy = vel_ctrl_h_y.compute(state[VEL_Y], 0.0);

    while (state[POS_Z] > 0.0) {
        const State  prev_state = state;
        const double prev_t     = t;
        ++sim_counter;

        // Get landing pad state
        Eigen::Vector2d pad_pos = landing_pad.pad_position(t);
        Eigen::Vector2d pad_vel = landing_pad.pad_velocity();

        // Vertical outer loop: position -> rate-limited ref_velocity in z-direction
        const double ref_vel_z = guidance.compute(state);

        // Gravity feedforward: hover thrust/force (in Newtons) that counteracts gravity at current mass
        const double feedforward = (MASS_DRY + state[FUEL]) * GRAVITY;

        // Vertical inner loop: velocity in z-direction -> thrust in z-direction
        const double Fz = vel_ctrl.compute(state[VEL_Z], ref_vel_z, feedforward);

        // Horizontal loops run at timestep_inner_h (slower than the sim step).
        // guidance_h.compute() is gated here so its internal iteration counter
        // increments at the correct rate for its outer_loop_divider to work.
        if (sim_counter % inner_loop_divider == 0) {
            ref_vel_xy = guidance_h.compute(state, pad_pos, pad_vel);
            Fx = vel_ctrl_h_x.compute(state[VEL_X], ref_vel_xy[0]);
            Fy = vel_ctrl_h_y.compute(state[VEL_Y], ref_vel_xy[1]);
        }

        // Assemble control input
        ControlInput control(3);
        control << Fx, Fy, Fz;

        // Get external disturbances
        Eigen::Vector3d wind_speeds = wind_model.wind_velocity(t);

        // Integrate dynamics one step
        SystemDynamics bound = [&](double time, const State& s) {
            return dynamics(time, s, control, wind_speeds);
        };
        state = Integrator::rk4_step(bound, t, state, dt_inner_v);
        t += dt_inner_v;

        // Touchdown interpolation: if we crossed the ground this step,
        // linearly interpolate to the exact contact event so the logged
        // landing velocity reflects the true contact condition.
        if (state[POS_Z] <= 0.0) {
            const double y0    = prev_state[POS_Z];
            const double y1    = state[POS_Z];
            const double denom = y0 - y1;
            double alpha = 1.0;
            if (denom > 0.0) alpha = std::clamp(y0 / denom, 0.0, 1.0);

            t = prev_t + alpha * dt_inner_v;
            state[POS_Z]  = 0.0;
            state[POS_X]  = prev_state[POS_X] + alpha * (state[POS_X] - prev_state[POS_X]);
            state[POS_Y]  = prev_state[POS_Y] + alpha * (state[POS_Y] - prev_state[POS_Y]); 
            state[VEL_Z]  = prev_state[VEL_Z]  + alpha * (state[VEL_Z]  - prev_state[VEL_Z]);
            state[VEL_X]  = prev_state[VEL_X]  + alpha * (state[VEL_X]  - prev_state[VEL_X]);
            state[VEL_Y]  = prev_state[VEL_Y]  + alpha * (state[VEL_Y]  - prev_state[VEL_Y]);
            state[FUEL] = prev_state[FUEL] + alpha * (state[FUEL] - prev_state[FUEL]);
        }

        state[FUEL] = std::max(0.0, state[FUEL]);

        logger.log(t, state, Fx, Fy, Fz, pad_pos[0], pad_pos[1], ref_vel_z, guidance.ref_vel_target());
    }
}

}  // namespace icarus
