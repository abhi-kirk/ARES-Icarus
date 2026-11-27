#pragma once

#include "dynamics/aero.h"
#include "dynamics/constants.h"
#include "dynamics/propulsion.h"

#include <algorithm>
#include <cmath>

namespace icarus {

// The landing laws already soft-land a hop near 400 m, falling at about
// 25 m/s, with almost no sideways speed and a few hundred kilograms of fuel.
// A 100 km coast with the engine off never arrives there: at 300 m it is
// still near 210 m/s down. This gate holds thrust at zero until a full-thrust
// brake can still reach that hop, and lights when waiting longer would miss it.

constexpr double ENTRY_HOP_VZ = -20.0;

// The jets need about 1.6 s to swing the nose from along the velocity onto
// the brake. Thrust stays off for that swing. Two more seconds of coast
// after that is the last moment the brake still reaches the hop.
constexpr double ENTRY_FLIP_S = 1.6;
constexpr double ENTRY_LOOKAHEAD_S = 2.0;

struct EntryPoint {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double mass;
};

struct EntryPlant {
    double gravity;
    double drag_scale;
    double thrust_max;
};

// True when this state is a hop PID, MPC, and SCvx already soft-land
// with the open 0 to 25 kN Earth plant. Weights are not part of this check.
inline bool entry_in_hop(const EntryPoint& point) {
    const double sideways = std::hypot(point.vx, point.vy);
    const double fuel = point.mass - MASS_DRY;
    const bool altitude = point.z >= 320.0 && point.z <= 520.0;
    const bool falling = point.vz >= -30.0 && point.vz <= -18.0;
    const bool slow_sideways = sideways < 10.0;
    const bool fueled = fuel >= 400.0;
    return altitude && falling && slow_sideways && fueled;
}

// Unit vector of the thrust that pulls the velocity toward (0, 0, -20 m/s).
inline void entry_brake_direction(
    double vx,
    double vy,
    double vz,
    double& ux,
    double& uy,
    double& uz
) {
    const double tx = -vx;
    const double ty = -vy;
    const double tz = ENTRY_HOP_VZ - vz;
    const double length = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (length < 1e-6) {
        ux = 0.0;
        uy = 0.0;
        uz = 1.0;
        return;
    }
    ux = tx / length;
    uy = ty / length;
    uz = tz / length;
}

// One mass-point step of the same drag law the 6-DOF plant uses.
// Attitude does not change drag. burn points the whole thrust ceiling
// along entry_brake_direction.
inline void entry_step(EntryPoint& point, const EntryPlant& plant, double dt, bool burn) {
    double drag_x = 0.0;
    double drag_y = 0.0;
    double drag_z = 0.0;
    aero_force(
        point.vx, point.vy, point.vz,
        0.0, 0.0, 0.0,
        point.z, plant.drag_scale, plant.gravity,
        drag_x, drag_y, drag_z
    );
    const double inv_mass = 1.0 / std::max(point.mass, 1.0);
    double ax = drag_x * inv_mass;
    double ay = drag_y * inv_mass;
    double az = drag_z * inv_mass - plant.gravity;

    if (burn && point.mass > MASS_DRY + 0.5 && plant.thrust_max > 1.0) {
        double ux = 0.0;
        double uy = 0.0;
        double uz = 1.0;
        entry_brake_direction(point.vx, point.vy, point.vz, ux, uy, uz);
        const double accel = plant.thrust_max / point.mass;
        ax += accel * ux;
        ay += accel * uy;
        az += accel * uz;
        const double flow = fuel_flow_kg_s(plant.thrust_max, EXHAUST_SPEED);
        point.mass = std::max(MASS_DRY, point.mass - flow * dt);
    }

    point.x += point.vx * dt;
    point.y += point.vy * dt;
    point.z += point.vz * dt + 0.5 * az * dt * dt;
    point.vx += ax * dt;
    point.vy += ay * dt;
    point.vz += az * dt;
}

inline bool entry_burn_reaches_hop(EntryPoint point, const EntryPlant& plant) {
    const double dt = 0.05;
    double t = 0.0;
    while (t < ENTRY_FLIP_S && point.z > 40.0) {
        entry_step(point, plant, dt, false);
        t += dt;
    }
    while (point.z > 40.0 && t < ENTRY_FLIP_S + 90.0 && point.mass > MASS_DRY + 0.5) {
        entry_step(point, plant, dt, true);
        t += dt;
        if (entry_in_hop(point)) return true;
    }
    return false;
}

// Light the flip when the brake still reaches the hop and another short
// coast would make that brake miss.
inline bool entry_should_flip(const EntryPoint& now, const EntryPlant& plant) {
    if (!entry_burn_reaches_hop(now, plant)) return false;
    EntryPoint later = now;
    double t = 0.0;
    const double dt = 0.05;
    while (t < ENTRY_LOOKAHEAD_S && later.z > 0.0) {
        entry_step(later, plant, dt, false);
        t += dt;
    }
    return !entry_burn_reaches_hop(later, plant);
}

}  // namespace icarus
