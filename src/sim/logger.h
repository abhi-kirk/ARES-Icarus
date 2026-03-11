#pragma once

#include <fstream>
#include <string>

#include "integrator.h"

namespace icarus {

// Logs simulation state to CSV and periodic stdout.
// CSV columns: Time, Altitude, Velocity, Fuel, Throttle
class Logger {
public:
    explicit Logger(const std::string& csv_path, int log_every_n = 10);
    ~Logger();

    void log(double t, const State& state, double throttle,
             double ref_vel_cmd, double ref_vel_target);

    void close();

private:
    std::ofstream csv_file_;
    int log_every_n_;
    int iteration_ = 0;
};

}  // namespace icarus
