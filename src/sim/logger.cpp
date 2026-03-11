#include "sim/logger.h"
#include "dynamics/state_indices.h"

#include <iomanip>
#include <iostream>

namespace icarus {

Logger::Logger(const std::string& csv_path, int log_every_n)
    : log_every_n_(log_every_n) {
    csv_file_.open(csv_path);
    csv_file_ << std::fixed << std::setprecision(6);
    csv_file_ << "Time,Altitude,Velocity,Fuel,Throttle\n";
}

Logger::~Logger() { close(); }

void Logger::log(double t, const State& state, double throttle,
                 double ref_vel_cmd, double ref_vel_target) {
    csv_file_ << t << "," << state[POS] << "," << state[VEL] << ","
              << state[FUEL] << "," << throttle << "\n";

    if (iteration_ % log_every_n_ == 0) {
        std::cout << "t=" << t
                  << " alt="     << state[POS]
                  << " vel="     << state[VEL]
                  << " ref_cmd=" << ref_vel_cmd
                  << " ref_tgt=" << ref_vel_target
                  << " thr="     << throttle << "\n";
    }
    ++iteration_;
}

void Logger::close() {
    if (csv_file_.is_open()) csv_file_.close();
}

}  // namespace icarus
