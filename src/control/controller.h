#pragma once

#include "config.h"
#include "integrator.h"

using std::string;

namespace icarus {
class Controller {
   private:
    double integral_feedback_error_ = 0.0;
    double previous_feedback_error_ = 0.0;
    double dt_;
    PIDConfig config_;

   public:
    Controller(double dt, const PIDConfig& config) : dt_(dt), config_(config) {}
    void reset_integral() { integral_feedback_error_ = 0.0; }
    void reset() {
        integral_feedback_error_ = 0.0;
        previous_feedback_error_ = 0.0;
    }

    double compute(const State& state, const string& measurement_type, const double& setpoint);
};

// specific controllers
double hoverslam_controller(const State& state);
double pid_controller(const State& state, const string& measurement_type, const double& setpoint,
                      double& integral_feedback_error, double& previous_feedback_error, double dt,
                      const PIDConfig& config);
}  // namespace icarus