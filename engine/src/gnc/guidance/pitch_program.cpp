#include "gnc/guidance/pitch_program.h"

#include "dynamics/aero.h"
#include "dynamics/state_indices.h"
#include "environment/gravity.h"
#include "gnc/guidance/ship.h"

#include <algorithm>
#include <cmath>

namespace icarus {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

struct LocalFlight {
    double altitude_m;
    double horizontal_m_s;
    double radial_m_s;
    double gamma_rad;
    double up_x;
    double up_y;
    double up_z;
    double east_x;
    double east_y;
    double east_z;
    double circular_m_s;
};

LocalFlight local_flight(const State& state) {
    const double x = state[POS_X];
    const double y = state[POS_Y];
    const double z = state[POS_Z];
    const double vx = state[VEL_X];
    const double vy = state[VEL_Y];
    const double vz = state[VEL_Z];
    const double radius = std::sqrt(x * x + y * y + z * z);
    const double inv_r = 1.0 / std::max(radius, 1.0);

    LocalFlight local;
    local.altitude_m = radius - EARTH_RADIUS_M;
    local.up_x = x * inv_r;
    local.up_y = y * inv_r;
    local.up_z = z * inv_r;

    const double radial = vx * local.up_x + vy * local.up_y + vz * local.up_z;
    const double hx = vx - radial * local.up_x;
    const double hy = vy - radial * local.up_y;
    const double hz = vz - radial * local.up_z;
    const double horizontal = std::sqrt(hx * hx + hy * hy + hz * hz);
    local.radial_m_s = radial;
    local.horizontal_m_s = horizontal;

    if (horizontal > 1.0) {
        local.east_x = hx / horizontal;
        local.east_y = hy / horizontal;
        local.east_z = hz / horizontal;
    } else {
        // No horizontal speed yet. Fall back to east in the pad sense,
        // with the up axis taken out so the two stay perpendicular.
        const double rx = 1.0 - local.up_x * local.up_x;
        const double ry = 0.0 - local.up_y * local.up_x;
        const double rz = 0.0 - local.up_z * local.up_x;
        const double ref = std::sqrt(rx * rx + ry * ry + rz * rz);
        if (ref > 1e-6) {
            local.east_x = rx / ref;
            local.east_y = ry / ref;
            local.east_z = rz / ref;
        } else {
            local.east_x = 0.0;
            local.east_y = 1.0;
            local.east_z = 0.0;
        }
    }

    local.gamma_rad = std::atan2(radial, std::max(horizontal, 1e-6));
    const double g = gravity_at_radius(GRAVITY_EARTH, radius);
    local.circular_m_s = std::sqrt(g * radius);
    return local;
}

bool near_circular(const LocalFlight& local) {
    const bool altitude = local.altitude_m > 180000.0 && local.altitude_m < 220000.0;
    const bool sideways = std::abs(local.horizontal_m_s - local.circular_m_s) < 80.0
        && local.horizontal_m_s > 7500.0;
    const bool flat = std::abs(local.gamma_rad) < 2.5 * kDeg;
    return altitude && sideways && flat;
}

}  // namespace

PitchAsk pitch_program(double t, const State& state, bool& inserted) {
    const LocalFlight local = local_flight(state);
    PitchAsk ask;
    ask.fx = 0.0;
    ask.fy = 0.0;
    ask.fz = 0.0;

    if (state[FUEL] <= SHIP_RESERVE_KG) {
        inserted = true;
    }
    if (!inserted && near_circular(local)) {
        inserted = true;
    }
    if (inserted) {
        return ask;
    }

    // Gravity turn. For the first few seconds the nose sits a little
    // above the velocity so the path bends toward the horizon. After
    // that the thrust rides the velocity.
    double pitch = local.gamma_rad;
    if (t < SHIP_KICK_S) {
        pitch += SHIP_KICK_DEG * kDeg;
    }
    const double along = std::cos(pitch);
    const double above = std::sin(pitch);
    const double dx = along * local.east_x + above * local.up_x;
    const double dy = along * local.east_y + above * local.up_y;
    const double dz = along * local.east_z + above * local.up_z;
    const double speed = std::sqrt(
        state[VEL_X] * state[VEL_X]
        + state[VEL_Y] * state[VEL_Y]
        + state[VEL_Z] * state[VEL_Z]
    );
    const double q = dynamic_pressure(local.altitude_m, speed, GRAVITY_EARTH);
    const double thrust = cap_thrust_for_q(SHIP_THRUST_N, q, SHIP_MAX_Q_PA);
    ask.fx = thrust * dx;
    ask.fy = thrust * dy;
    ask.fz = thrust * dz;
    return ask;
}

}  // namespace icarus
