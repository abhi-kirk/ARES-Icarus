#include "config/config.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "json.hpp"

using json = nlohmann::json;

namespace icarus {

bool load_config(const std::string& filepath, Config& config) {
    try {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open config file: " << filepath << std::endl;
            return false;
        }

        json j = json::parse(file);

        config.pid.kp_v_vel = j["inner_pid_vertical_velocity"]["gains"]["kp"];
        config.pid.kd_v_vel = j["inner_pid_vertical_velocity"]["gains"]["kd"];
        config.pid.ki_v_vel = j["inner_pid_vertical_velocity"]["gains"]["ki"];

        config.pid.kp_v_pos = j["outer_pid_vertical_position"]["gains"]["kp"];
        config.pid.kd_v_pos = j["outer_pid_vertical_position"]["gains"]["kd"];
        config.pid.ki_v_pos = j["outer_pid_vertical_position"]["gains"]["ki"];

        config.pid.kp_h_vel = j["inner_pid_horizontal_velocity"]["gains"]["kp"];
        config.pid.kd_h_vel = j["inner_pid_horizontal_velocity"]["gains"]["kd"];
        config.pid.ki_h_vel = j["inner_pid_horizontal_velocity"]["gains"]["ki"];

        config.pid.kp_h_pos = j["outer_pid_horizontal_position"]["gains"]["kp"];
        config.pid.kd_h_pos = j["outer_pid_horizontal_position"]["gains"]["kd"];
        config.pid.ki_h_pos = j["outer_pid_horizontal_position"]["gains"]["ki"];

        config.simulation.timestep_inner     = j["simulation"]["timestep_inner"];
        config.simulation.timestep_outer     = j["simulation"]["timestep_outer"];
        config.simulation.timestep_inner_h   = j["simulation"]["timestep_inner_h"];
        config.simulation.timestep_outer_h   = j["simulation"]["timestep_outer_h"];

        config.simulation.initial_px         = j["simulation"]["initial_x_pos"];
        config.simulation.initial_py         = j["simulation"]["initial_y_pos"];
        config.simulation.initial_pz         = j["simulation"]["initial_z_pos"];

        config.simulation.initial_vx         = j["simulation"]["initial_x_velocity"];
        config.simulation.initial_vy         = j["simulation"]["initial_y_velocity"];
        config.simulation.initial_vz         = j["simulation"]["initial_z_velocity"];

        config.simulation.pad_initial_x      = j["simulation"].value("initial_pad_x_pos", 0.0);
        config.simulation.pad_initial_y      = j["simulation"].value("initial_pad_y_pos", 0.0);
        config.simulation.pad_drift_vx       = j["simulation"].value("initial_pad_x_velocity", 0.0);
        config.simulation.pad_drift_vy       = j["simulation"].value("initial_pad_y_velocity", 0.0);

        config.simulation.wind_constant_x    = j["simulation"].value("wind_x_velocity", 0.0);
        config.simulation.wind_constant_y    = j["simulation"].value("wind_y_velocity", 0.0);
        config.simulation.gust_magnitude     = j["simulation"].value("gust_magnitude", 0.0);
        config.simulation.gust_frequency     = j["simulation"].value("gust_frequency", 0.0);

        config.simulation.max_ref_accel_v      = j["simulation"].value("max_ref_accel_vertical", 2.0);
        config.simulation.max_ref_accel_h      = j["simulation"].value("max_ref_accel_horizontal", 2.0);

        config.simulation.initial_fuel       = j["simulation"]["initial_fuel"];
        config.simulation.use_rerun          = j["simulation"].value("use_rerun", false);
        config.simulation.rerun_spawn_viewer = j["simulation"].value("rerun_spawn_viewer", true);
        config.simulation.rerun_save_rrd     = j["simulation"].value("rerun_save_rrd", true);

        std::cout << "Config loaded from: " << filepath << std::endl;
        return true;
    } catch (const json::exception& e) {
        std::cerr << "JSON parsing error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace icarus
