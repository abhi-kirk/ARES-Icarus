#pragma once

#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include <rerun.hpp>

#include "integrator.h"

namespace icarus {

// Logs simulation state to CSV, periodic stdout, and optionally Rerun.
// CSV columns: Time, Altitude, Velocity, Fuel, Throttle
class Logger {
public:
    explicit Logger(const std::string& csv_path,
                    bool use_rerun          = false,
                    bool rerun_spawn_viewer = true,
                    bool rerun_save_rrd     = true,
                    const std::string& rrd_path = "src/results/trajectory.rrd",
                    int log_every_n         = 10);
    ~Logger();

    void log(double t, const State& state, double throttle,
             double ref_vel_cmd, double ref_vel_target);

    void close();

private:
    std::ofstream csv_file_;
    int log_every_n_;
    int iteration_ = 0;

    bool use_rerun_;
    std::optional<rerun::RecordingStream> rec_;
    std::vector<rerun::datatypes::Vec3D> trail_points_;
};

}  // namespace icarus
