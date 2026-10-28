#pragma once

#include <atomic>
#include <functional>

#include "config/config.h"
#include "gnc/control/controller.h"
#include "gnc/guidance/cascade_guidance.h"
#include "gnc/guidance/horizontal_guidance.h"
#include "environment/wind_model.h"
#include "environment/landing_pad.h"
#include "integrator.h"
#include "sim/logger.h"

namespace icarus {

// f(t, state, control inputs, external disturbances) -> state_dot
using DynamicsFunction = std::function<State(double, const State&, const ControlInput&, const Disturbance&)>;

// One row of a landing. The page plays these while the flight is running.
struct FlightSample {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double fuel = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double pad_x = 0.0;
    double pad_y = 0.0;
};

using SampleEmit = std::function<void(const FlightSample&)>;

class Simulator {
public:
    // Flies until altitude reaches zero, the stop flag is set, or time_limit
    // is passed. time_limit of 0 means no time cap. logger may be null when the
    // caller only wants samples. emit may be null when the caller only wants the file.
    void run(
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
        Logger* logger,
        const std::atomic<bool>* stop,
        const SampleEmit* emit,
        double time_limit
    );
};

}  // namespace icarus
