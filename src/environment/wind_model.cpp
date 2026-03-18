#include "environment/wind_model.h"
#include <cmath>

namespace icarus {

WindModel::WindModel(const SimConfig& config) {
    constant_x_ = config.wind_constant_x;
    constant_y_ = config.wind_constant_y;
    gust_magnitude_ = config.gust_magnitude;
    gust_frequency_ = config.gust_frequency;
}

Eigen::Vector3d WindModel::wind_velocity(double t) const {
    Eigen::Vector3d wind_vel;

    double gust = gust_magnitude_ * std::sin(2 * M_PI * gust_frequency_ * t);

    double wx = constant_x_ + gust;
    double wy = constant_y_ + gust;
    double wz = 0.0;

    wind_vel << wx, wy, wz;

    return wind_vel;
}

}  // namespace icarus