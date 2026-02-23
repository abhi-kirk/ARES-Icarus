#include "controller.h"
#include "integrator.h"
#include "state_indices.h"

using icarus::State;

namespace icarus {
// Simple Hoverslam Logic: A rapid, precise burn to kill vertical velocity just before
// touchdown, famously used by SpaceX.
double hoverslam_controller(const State& state) {
    double throttle = 0.0;
    if (state[POS] < 500.0 && state[VEL] < -5.0) {
        throttle = 1.0;
    }

    return throttle;
}
}  // namespace icarus