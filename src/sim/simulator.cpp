#include "sim/simulator.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"

#include <algorithm>
#include <cmath>

namespace icarus {

void Simulator::run(State& state, double t_start,
                    const SimConfig& cfg,
                    CascadeGuidance& guidance,
                    Controller& vel_ctrl,
                    const ThrottledDynamics& dynamics,
                    Logger& logger) {
    double t  = t_start;
    double dt = cfg.timestep_inner;

    while (state[POS] > 0.0) {
        const State  prev_state = state;
        const double prev_t     = t;

        // Outer loop: position -> rate-limited ref_velocity
        const double ref_vel_cmd = guidance.compute(state);

        // Gravity feedforward: hover throttle that counteracts gravity at current mass
        const double feedforward = (MASS_DRY + state[FUEL]) * GRAVITY / MAX_THRUST;

        // Inner loop: velocity -> throttle
        const double throttle = vel_ctrl.compute(state[VEL], ref_vel_cmd, feedforward);

        // Integrate dynamics one step
        SystemDynamics bound = [&](double time, const State& s) {
            return dynamics(time, s, throttle);
        };
        state = Integrator::rk4_step(bound, t, state, dt);
        t += dt;

        // Touchdown interpolation: if we crossed the ground this step,
        // linearly interpolate to the exact contact event so the logged
        // landing velocity reflects the true contact condition.
        if (state[POS] <= 0.0) {
            const double y0    = prev_state[POS];
            const double y1    = state[POS];
            const double denom = y0 - y1;
            double alpha = 1.0;
            if (denom > 0.0) alpha = std::clamp(y0 / denom, 0.0, 1.0);

            t           = prev_t + alpha * dt;
            state[POS]  = 0.0;
            state[VEL]  = prev_state[VEL]  + alpha * (state[VEL]  - prev_state[VEL]);
            state[FUEL] = prev_state[FUEL] + alpha * (state[FUEL] - prev_state[FUEL]);
        }

        state[FUEL] = std::max(0.0, state[FUEL]);

        logger.log(t, state, throttle, ref_vel_cmd, guidance.ref_vel_target());
    }
}

}  // namespace icarus
