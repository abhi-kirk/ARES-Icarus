#include "sim/logger.h"
#include "dynamics/state_indices.h"

#include <iomanip>
#include <iostream>

namespace icarus {

Logger::Logger(const std::string& csv_path,
               bool use_rerun,
               bool rerun_spawn_viewer,
               bool rerun_save_rrd,
               const std::string& rrd_path,
               int log_every_n)
    : log_every_n_(log_every_n), use_rerun_(use_rerun) {
    csv_file_.open(csv_path);
    csv_file_ << std::fixed << std::setprecision(6);
    csv_file_ << "Time,Altitude,Velocity,Fuel,Throttle\n";

    if (use_rerun_) {
        rec_.emplace("icarus");

        bool viewer_ok = false;
        if (rerun_spawn_viewer) {
            if (auto err = rerun::spawn(); !err.is_ok()) {
                std::cerr << "[rerun] spawn failed (is `rerun` in PATH?): "
                          << err.description << "\n";
            } else {
                viewer_ok = true;
            }
        }

        // set_sinks() replaces all sinks at once — supports simultaneous file + viewer
        if (rerun_save_rrd && viewer_ok) {
            rec_->set_sinks(rerun::FileSink{rrd_path}, rerun::GrpcSink{}).exit_on_failure();
        } else if (rerun_save_rrd) {
            rec_->set_sinks(rerun::FileSink{rrd_path}).exit_on_failure();
        } else if (viewer_ok) {
            rec_->set_sinks(rerun::GrpcSink{}).exit_on_failure();
        }

        rec_->log_static("world", rerun::archetypes::ViewCoordinates::RIGHT_HAND_Z_UP);
    }
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

        if (use_rerun_) {
            rec_->set_time_duration_secs("sim_time", t);

            float alt = static_cast<float>(state[POS]);
            float vel = static_cast<float>(state[VEL]);

            // 3D rocket position (moves along Z axis only for 1-DOF)
            trail_points_.push_back({0.0f, 0.0f, alt});
            rec_->log("rocket/position",
                rerun::Points3D({{0.0f, 0.0f, alt}})
                    .with_radii({2.0f})
                    .with_colors({rerun::Color(255, 100, 0)}));

            // Thrust vector arrow (scaled by throttle)
            float thrust_viz = static_cast<float>(throttle) * 30.0f;
            rec_->log("rocket/thrust",
                rerun::Arrows3D::from_vectors({{0.0f, 0.0f, thrust_viz}})
                    .with_origins({{0.0f, 0.0f, alt}})
                    .with_colors({rerun::Color(255, 200, 0)}));

            // Trail of past positions
            rec_->log("rocket/trail",
                rerun::LineStrips3D({rerun::components::LineStrip3D(trail_points_)}));

            // Scalar time-series panels
            rec_->log("telemetry/altitude",    rerun::Scalars(alt));
            rec_->log("telemetry/velocity",    rerun::Scalars(vel));
            rec_->log("telemetry/throttle",    rerun::Scalars(static_cast<float>(throttle)));
            rec_->log("telemetry/ref_vel_cmd", rerun::Scalars(static_cast<float>(ref_vel_cmd)));
        }
    }

    ++iteration_;
}

void Logger::close() {
    if (csv_file_.is_open()) csv_file_.close();
}

}  // namespace icarus
