#include "config.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "json.hpp"

using json = nlohmann::json;

namespace icarus {

bool load_config(const std::string& filepath, Config& config) {
    try {
        // Open and parse JSON file
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open config file: " << filepath << std::endl;
            return false;
        }

        json j = json::parse(file);

        // Load PID configuration
        config.pid.kp_vel = j["pid_velocity"]["gains"]["kp"];
        config.pid.kd_vel = j["pid_velocity"]["gains"]["kd"];
        config.pid.ki_vel = j["pid_velocity"]["gains"]["ki"];

        config.pid.kp_pos = j["pid_position"]["gains"]["kp"];
        config.pid.kd_pos = j["pid_position"]["gains"]["kd"];
        config.pid.ki_pos = j["pid_position"]["gains"]["ki"];

        // Load simulation configuration
        config.simulation.timestep_inner = j["simulation"]["timestep_inner"];
        config.simulation.timestep_outer = j["simulation"]["timestep_outer"];
        config.simulation.initial_altitude = j["simulation"]["initial_altitude"];
        config.simulation.initial_velocity = j["simulation"]["initial_velocity"];
        config.simulation.initial_fuel = j["simulation"]["initial_fuel"];

        std::cout << "Config loaded successfully from: " << filepath << std::endl;
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
