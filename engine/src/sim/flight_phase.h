#pragma once

#include "config/config.h"
#include "dynamics/attitude.h"
#include "integrator.h"

namespace icarus {

// Coast holds the engine off and the nose on the velocity. Flip keeps the
// engine off while the jets swing the nose onto the brake. Brake is the
// ceiling thrust toward a 20 m/s fall. Land is the existing law.
enum class EntryPhase {
    Coast,
    Flip,
    Brake,
    Land
};

struct EntryAsk {
    bool guide = false;
    bool force_solve = false;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    Quat attitude;
};

// Advances the coast, the flip, or the brake by one step. Land is left
// to the caller, which runs PID, MPC, or SCvx.
EntryAsk update_entry(
    EntryPhase& phase,
    double& check_age,
    double dt,
    double t,
    const State& state,
    const SimConfig& cfg,
    Quat attitude_now
);

}  // namespace icarus
