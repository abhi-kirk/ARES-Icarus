#pragma once

#include "integrator.h"

namespace icarus {

// Inertial force from the ascent pitch program. Zero once the ship is
// on a circular orbit, or once fuel is down to the landing reserve.
struct PitchAsk {
    double fx;
    double fy;
    double fz;
};

// inserted latches the cutoff. A coast that stays near circular does not
// light the engine again.
PitchAsk pitch_program(double t, const State& state, bool& inserted);

}  // namespace icarus
