#include "gnc/guidance/soft_land_reference.h"

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>

namespace icarus {

void SoftLandReference::build(
    const Eigen::Vector3d& pos0,
    const Eigen::Vector3d& vel0,
    double mass,
    double thrust_min,
    double thrust_max,
    double dt,
    double pad_x,
    double pad_y
) {
    knots_.clear();
    dt_ = std::max(0.05, dt);

    // Work in pad-relative coordinates so the target is the origin.
    const double x0 = pos0.x() - pad_x;
    const double y0 = pos0.y() - pad_y;
    const double z0 = std::max(1.0, pos0.z());
    const double vx0 = vel0.x();
    const double vy0 = vel0.y();
    const double vz0 = vel0.z();

    // Vertical soft-land time for constant accel: zf = z0 + vz0*tf + 0.5*a*tf^2 = 0
    // and vzf = vz0 + a*tf = 0 => a = -vz0/tf => tf = -2*z0/vz0 when descending.
    double tf = 20.0;
    if (vz0 < -0.5) {
        tf = -2.0 * z0 / vz0;
    } else {
        // Hovering or climbing: pick a time that can brake under thrust_max.
        const double a_down = GRAVITY - thrust_max / std::max(mass, 1.0);
        // Use a mild descent accel of about 2 m/s^2 net down if possible.
        const double a_cmd = std::min(-2.0, a_down);
        tf = std::sqrt(std::max(1.0, -2.0 * z0 / a_cmd));
    }
    tf = std::clamp(tf, 8.0, 80.0);

    // Net vertical accel that reaches z=0, vz=0 in tf (exact for constant a).
    double a_z = -2.0 * (z0 / (tf * tf) + vz0 / tf);
    // Horizontal: cancel position and velocity by tf under constant accel.
    double a_x = -2.0 * x0 / (tf * tf) - 2.0 * vx0 / tf;
    double a_y = -2.0 * y0 / (tf * tf) - 2.0 * vy0 / tf;

    // Convert accel to force. Fz fights gravity as well.
    auto force_from_accel = [&](double ax, double ay, double az) {
        Eigen::Vector3d F;
        F.x() = mass * ax;
        F.y() = mass * ay;
        F.z() = mass * (az + GRAVITY);
        // Keep magnitude inside the thrust ring (isotropic clamp).
        double mag = F.norm();
        if (mag > thrust_max && mag > 1e-9) {
            F *= thrust_max / mag;
        }
        if (mag < thrust_min && mag > 1e-9) {
            F *= thrust_min / mag;
        }
        if (mag <= 1e-9 && thrust_min > 0.0) {
            F = Eigen::Vector3d(0.0, 0.0, thrust_min);
        }
        return F;
    };

    Eigen::Vector3d F0 = force_from_accel(a_x, a_y, a_z);
    // Recompute accel from the clamped force so the knot trail matches the ask.
    a_x = F0.x() / mass;
    a_y = F0.y() / mass;
    a_z = F0.z() / mass - GRAVITY;

    duration_ = tf;
    const int steps = std::max(2, static_cast<int>(std::ceil(tf / dt_)));
    const double step = tf / steps;

    for (int k = 0; k <= steps; ++k) {
        const double t = k * step;
        ReferenceKnot knot;
        knot.t = t;
        // Pad-relative position and velocity under constant accel.
        const double xr = x0 + vx0 * t + 0.5 * a_x * t * t;
        const double yr = y0 + vy0 * t + 0.5 * a_y * t * t;
        const double zr = z0 + vz0 * t + 0.5 * a_z * t * t;
        knot.pos = Eigen::Vector3d(xr + pad_x, yr + pad_y, std::max(0.0, zr));
        knot.vel = Eigen::Vector3d(vx0 + a_x * t, vy0 + a_y * t, vz0 + a_z * t);
        if (knot.pos.z() <= 1e-6) {
            knot.pos.z() = 0.0;
            knot.vel = Eigen::Vector3d::Zero();
        }
        knot.force = F0;
        knots_.push_back(knot);
    }
}

ReferenceKnot SoftLandReference::at(double t) const {
    if (knots_.empty()) return ReferenceKnot{};
    if (t <= knots_.front().t) return knots_.front();
    if (t >= knots_.back().t) return knots_.back();

    // Linear blend between the two knots that bracket t.
    for (size_t i = 1; i < knots_.size(); ++i) {
        if (t <= knots_[i].t) {
            const ReferenceKnot& a = knots_[i - 1];
            const ReferenceKnot& b = knots_[i];
            const double span = b.t - a.t;
            double alpha = 0.0;
            if (span > 1e-12) alpha = (t - a.t) / span;
            ReferenceKnot out;
            out.t = t;
            out.pos = a.pos + alpha * (b.pos - a.pos);
            out.vel = a.vel + alpha * (b.vel - a.vel);
            out.force = a.force + alpha * (b.force - a.force);
            return out;
        }
    }
    return knots_.back();
}

}  // namespace icarus
