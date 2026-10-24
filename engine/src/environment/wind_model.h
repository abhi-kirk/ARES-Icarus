#pragma once

#include "config/config.h"
#include <Eigen/Dense>

namespace icarus {

class WindModel {
public:
    explicit WindModel(const SimConfig& config);

    // Returns wind velocity vector [wx, wy, wz] at time t
    Eigen::Vector3d wind_velocity(double t) const;

private:
    double constant_x_;
    double constant_y_;
    double gust_magnitude_;
    double gust_frequency_;
};

}  // namespace icarus