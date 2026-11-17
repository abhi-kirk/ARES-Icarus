#include "gnc/guidance/soft_land_reference.h"

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>

namespace icarus {

void SoftLandReference::build(
    double t0,
    const Eigen::Vector3d& pos0,
    const Eigen::Vector3d& vel0,
    double mass,
    double thrust_min,
    double thrust_max,
    double pointing_cone_deg,
    double dt,
    double pad_x,
    double pad_y
) {
    knots_.clear();
    t0_ = t0;
    mass_used_ = std::max(mass, 1.0);
    dt_ = std::max(0.05, dt);

    // Work in pad-relative coordinates so the target is the origin.
    const double x0 = pos0.x() - pad_x;
    const double y0 = pos0.y() - pad_y;
    const double z_now = std::max(1.0, pos0.z());
    const double vx0 = vel0.x();
    const double vy0 = vel0.y();
    const double vz_now = vel0.z();
    const double m = mass_used_;

    // Soft-land time tf = -2*z/vz only works while descending. If the vehicle is
    // climbing or nearly hovering, map the state to an effective apex and a
    // small downward seed so the constant-accel profile still reaches the pad.
    double z0 = z_now;
    double vz0 = vz_now;
    if (vz_now >= -0.5) {
        if (vz_now > 0.0) {
            z0 = z_now + (vz_now * vz_now) / (2.0 * GRAVITY);
        }
        vz0 = -0.5;
    }
    z0 = std::max(1.0, z0);

    // Vertical soft-land time for constant accel: zf = z0 + vz0*tf + 0.5*a*tf^2 = 0
    // and vzf = vz0 + a*tf = 0 => a = -vz0/tf => tf = -2*z0/vz0 when descending.
    double tf_z = -2.0 * z0 / vz0;
    const double a_down = GRAVITY - thrust_max / m;
    const double a_cmd = std::min(-2.0, a_down);
    const double tf_ballistic = std::sqrt(std::max(1.0, -2.0 * z0 / a_cmd));
    tf_z = std::max(tf_z, tf_ballistic);
    // Cap tf so a slow current vz cannot stretch the profile into a crawl
    // (e.g. z=140 m at vz=-2.5 gave tf≈112 s and burned the clock).
    tf_z = std::min(tf_z, std::max(5.0, z0 / 5.0));
    tf_z = std::clamp(tf_z, 5.0, 80.0);

    // Lateral time is shorter than vertical so XY reaches the pad before z
    // does. Sharing one long tf left ~20 m of miss at soft contact under wind.
    const double range_xy = std::hypot(x0, y0);
    double tf_xy = tf_z;
    if (range_xy > 1.0) {
        tf_xy = std::min(tf_z, std::max(5.0, range_xy / 25.0));
    }

    // Once laterally acquired: firm descent while high, then soften in the
    // last tens of meters so touchdown |vz| stays inside the soft gate.
    if (range_xy < 10.0) {
        if (z_now > 25.0) {
            tf_z = std::min(tf_z, std::max(4.0, z_now / 5.0));
        } else {
            tf_z = std::min(tf_z, std::max(6.0, z_now / 1.5));
        }
    }

    const double tf = std::max(tf_z, tf_xy);

    // Profile accelerations use the live velocity so the first segment matches
    // the vehicle (including a climb that the effective apex absorbed in tf_z).
    double a_z = -2.0 * (z_now / (tf_z * tf_z) + vz_now / tf_z);
    double a_x = -2.0 * x0 / (tf_xy * tf_xy) - 2.0 * vx0 / tf_xy;
    double a_y = -2.0 * y0 / (tf_xy * tf_xy) - 2.0 * vy0 / tf_xy;

    const double point_rad = pointing_cone_deg * (3.14159265358979323846 / 180.0);
    const double tan_point =
        (pointing_cone_deg >= 89.9) ? 1.0e6 : std::tan(point_rad);

    auto force_from_accel = [&](double ax, double ay, double az) {
        Eigen::Vector3d F;
        F.x() = m * ax;
        F.y() = m * ay;
        F.z() = m * (az + GRAVITY);
        // Keep vertical braking first when the pointing cone is tight: clip
        // side force to tan(point)*Fz instead of shrinking the whole vector.
        if (pointing_cone_deg < 89.9 && F.z() > 1e-9) {
            const double lat_max = tan_point * F.z();
            const double lat = std::hypot(F.x(), F.y());
            if (lat > lat_max && lat > 1e-9) {
                F.x() *= lat_max / lat;
                F.y() *= lat_max / lat;
            }
        }
        double mag = F.norm();
        if (mag > thrust_max && mag > 1e-9) {
            // Under a tight ceiling, protect Fz (braking) and cut lateral.
            if (F.z() > thrust_max) {
                F.z() = thrust_max;
                F.x() = 0.0;
                F.y() = 0.0;
            } else {
                const double lat_budget = std::sqrt(
                    std::max(0.0, thrust_max * thrust_max - F.z() * F.z())
                );
                const double lat = std::hypot(F.x(), F.y());
                if (lat > lat_budget && lat > 1e-9) {
                    F.x() *= lat_budget / lat;
                    F.y() *= lat_budget / lat;
                }
            }
            mag = F.norm();
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
    a_x = F0.x() / m;
    a_y = F0.y() / m;
    a_z = F0.z() / m - GRAVITY;

    duration_ = tf;
    const int steps = std::max(2, static_cast<int>(std::ceil(tf / dt_)));
    const double step = tf / steps;

    // Hover ask used once the profile reaches the pad. Keeping the initial
    // deceleration force F0 on the grounded knots makes the MPC bounce.
    const Eigen::Vector3d F_hover(0.0, 0.0, m * GRAVITY);

    for (int k = 0; k <= steps; ++k) {
        const double tau = k * step;
        ReferenceKnot knot;
        knot.t = t0_ + tau;
        const double xr = x0 + vx0 * tau + 0.5 * a_x * tau * tau;
        const double yr = y0 + vy0 * tau + 0.5 * a_y * tau * tau;
        const double zr = z_now + vz_now * tau + 0.5 * a_z * tau * tau;
        knot.pos = Eigen::Vector3d(xr + pad_x, yr + pad_y, std::max(0.0, zr));
        knot.vel = Eigen::Vector3d(vx0 + a_x * tau, vy0 + a_y * tau, vz_now + a_z * tau);
        knot.force = F0;
        if (knot.pos.z() <= 1e-6) {
            knot.pos.z() = 0.0;
            knot.vel = Eigen::Vector3d::Zero();
            knot.force = F_hover;
        }
        knots_.push_back(knot);
    }

    // Guarantee the final knot is a settled hover on the pad.
    if (!knots_.empty()) {
        ReferenceKnot& last = knots_.back();
        last.pos = Eigen::Vector3d(pad_x, pad_y, 0.0);
        last.vel = Eigen::Vector3d::Zero();
        last.force = F_hover;
    }
}

ReferenceKnot SoftLandReference::at(double t) const {
    if (knots_.empty()) return ReferenceKnot{};
    if (t <= knots_.front().t) return knots_.front();
    if (t >= knots_.back().t) return knots_.back();

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
