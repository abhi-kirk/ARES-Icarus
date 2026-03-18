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
    csv_file_ << "Time,PosX,PosY,PosZ,VelX,VelY,VelZ,Fuel,Fx,Fy,Fz\n";

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

        // Set line width and explicit colors for all time-series channels.
        // Colors are set here so they are consistent across viewer restarts
        // and in the published .rrd — Rerun auto-assigns colors otherwise,
        // which can produce nearly identical hues for adjacent series.
        float lw = 2.0f;
        rec_->log_static("telemetry/altitude",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color(255, 120,  50)));  // orange
        rec_->log_static("telemetry/vel_z",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color( 80, 220,  80)));  // green
        rec_->log_static("telemetry/ref_vel_cmd",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color( 80, 160, 255)));  // blue
        rec_->log_static("telemetry/vel_x",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color(255, 200,  50)));  // yellow
        rec_->log_static("telemetry/vel_y",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color(220,  80, 220)));  // magenta
        rec_->log_static("telemetry/Fz",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color( 80, 220, 220)));  // cyan
        rec_->log_static("telemetry/Fx",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color(255, 200,  50)));  // yellow
        rec_->log_static("telemetry/Fy",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color(220,  80, 220)));  // magenta
        rec_->log_static("telemetry/fuel",
            rerun::SeriesLines().with_widths(lw).with_colors(rerun::Color(255,  80,  80)));  // red
    }
}

Logger::~Logger() { close(); }

void Logger::log(double t, const State& state, double Fx, double Fy, double Fz,
                 double pad_x, double pad_y,
                 double ref_vel_cmd, double ref_vel_target) {
    csv_file_ << t
              << "," << state[POS_X] << "," << state[POS_Y] << "," << state[POS_Z]
              << "," << state[VEL_X] << "," << state[VEL_Y] << "," << state[VEL_Z]
              << "," << state[FUEL]
              << "," << Fx << "," << Fy << "," << Fz
              << "\n";

    if (iteration_ % log_every_n_ == 0) {
        std::cout << "t=" << t
                  << " pos=[" << state[POS_X] << "," << state[POS_Y] << "," << state[POS_Z] << "]"
                  << " vel=[" << state[VEL_X] << "," << state[VEL_Y] << "," << state[VEL_Z] << "]"
                  << " ref_cmd=" << ref_vel_cmd
                  << " ref_tgt=" << ref_vel_target
                  << " F=[" << Fx << "," << Fy << "," << Fz << "]"
                  << "\n";

        if (use_rerun_) {
            rec_->set_time_duration_secs("sim_time", t);

            float px  = static_cast<float>(state[POS_X]);
            float py  = static_cast<float>(state[POS_Y]);
            float alt = static_cast<float>(state[POS_Z]);

            // 3D rocket position
            trail_points_.push_back({px, py, alt});
            rec_->log("rocket/position",
                rerun::Points3D({{px, py, alt}})
                    .with_radii({2.0f})
                    .with_colors({rerun::Color(255, 100, 0)}));

            // Thrust vector arrow (scaled for visibility)
            float thrust_scale = 30.0f / 25000.0f;  // normalize by MAX_THRUST
            float fx_viz = static_cast<float>(Fx) * thrust_scale;
            float fy_viz = static_cast<float>(Fy) * thrust_scale;
            float fz_viz = static_cast<float>(Fz) * thrust_scale;
            rec_->log("rocket/thrust",
                rerun::Arrows3D::from_vectors({{fx_viz, fy_viz, fz_viz}})
                    .with_origins({{px, py, alt}})
                    .with_colors({rerun::Color(255, 200, 0)}));

            // Landing pad position (moves over time)
            float lpx = static_cast<float>(pad_x);
            float lpy = static_cast<float>(pad_y);
            rec_->log("world/pad",
                rerun::Points3D({{lpx, lpy, 0.0f}})
                    .with_radii({5.0f})
                    .with_colors({rerun::Color(0, 220, 220)}));

            // Trail of past positions
            rec_->log("rocket/trail",
                rerun::LineStrips3D({rerun::components::LineStrip3D(trail_points_)}));

            // Scalar time-series panels
            rec_->log("telemetry/altitude",    rerun::Scalars(alt));
            rec_->log("telemetry/vel_z",       rerun::Scalars(static_cast<float>(state[VEL_Z])));
            rec_->log("telemetry/vel_x",       rerun::Scalars(static_cast<float>(state[VEL_X])));
            rec_->log("telemetry/vel_y",       rerun::Scalars(static_cast<float>(state[VEL_Y])));
            rec_->log("telemetry/Fz",          rerun::Scalars(static_cast<float>(Fz)));
            rec_->log("telemetry/Fx",          rerun::Scalars(static_cast<float>(Fx)));
            rec_->log("telemetry/Fy",          rerun::Scalars(static_cast<float>(Fy)));
            rec_->log("telemetry/fuel",        rerun::Scalars(static_cast<float>(state[FUEL])));
            rec_->log("telemetry/ref_vel_cmd", rerun::Scalars(static_cast<float>(ref_vel_cmd)));
        }
    }

    ++iteration_;
}

void Logger::close() {
    if (csv_file_.is_open()) csv_file_.close();
}

}  // namespace icarus
