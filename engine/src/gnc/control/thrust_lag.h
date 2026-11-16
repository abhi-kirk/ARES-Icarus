#pragma once

#include <algorithm>
#include <cmath>

namespace icarus {

// First-order engine lag: Ḟ = (u − F) / τ.
// Exact discrete step for a held command over dt. When tau is ~0 the
// engine matches the command immediately (old Phase 2 behavior).
inline double lag_step(double actual, double command, double dt, double tau) {
    if (tau <= 1e-9 || dt <= 0.0) return command;
    const double alpha = 1.0 - std::exp(-dt / tau);
    return actual + alpha * (command - actual);
}

// Optional hard slew on top of the lag: |ΔF| / dt cannot exceed slew_n_per_s.
// slew <= 0 leaves the lagged value alone.
inline double slew_limit(double previous, double proposed, double dt, double slew_n_per_s) {
    if (slew_n_per_s <= 0.0 || dt <= 0.0) return proposed;
    const double max_delta = slew_n_per_s * dt;
    return previous + std::clamp(proposed - previous, -max_delta, max_delta);
}

inline void apply_thrust_lag(
    double& Fx_act,
    double& Fy_act,
    double& Fz_act,
    double Fx_cmd,
    double Fy_cmd,
    double Fz_cmd,
    double dt,
    double tau,
    double slew_n_per_s
) {
    const double fx_lag = lag_step(Fx_act, Fx_cmd, dt, tau);
    const double fy_lag = lag_step(Fy_act, Fy_cmd, dt, tau);
    const double fz_lag = lag_step(Fz_act, Fz_cmd, dt, tau);

    Fx_act = slew_limit(Fx_act, fx_lag, dt, slew_n_per_s);
    Fy_act = slew_limit(Fy_act, fy_lag, dt, slew_n_per_s);
    Fz_act = slew_limit(Fz_act, fz_lag, dt, slew_n_per_s);
}

}  // namespace icarus
