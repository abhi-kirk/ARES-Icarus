#include "sim/flight_phase.h"

#include "dynamics/attitude.h"
#include "dynamics/constants.h"
#include "dynamics/state_indices.h"
#include "environment/frames.h"
#include "gnc/guidance/entry_coast.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace icarus {

EntryAsk update_entry(
    EntryPhase& phase,
    double& check_age,
    double dt,
    double t,
    const State& state,
    const SimConfig& cfg,
    Quat attitude_now
) {
    EntryAsk ask;
    ask.guide = false;
    ask.force_solve = false;
    ask.fx = 0.0;
    ask.fy = 0.0;
    ask.fz = 0.0;
    ask.attitude = attitude_now;

    EntryPlant plant;
    plant.gravity = cfg.gravity;
    plant.drag_scale = cfg.drag_scale;
    plant.thrust_max = cfg.thrust_max;

    // A ship state is meters from Earth's center. The hop box and the
    // brake are written in the pad frame, so convert before the check.
    // The tangent frame at the pad shares this file's x, y, z axes.
    double px = state[POS_X];
    double py = state[POS_Y];
    double pz = state[POS_Z];
    double pvx = state[VEL_X];
    double pvy = state[VEL_Y];
    double pvz = state[VEL_Z];
    if (cfg.planet_centered) {
        const PadState pad = local_state(px, py, pz, pvx, pvy, pvz);
        px = pad.x;
        py = pad.y;
        pz = pad.z;
        pvx = pad.vx;
        pvy = pad.vy;
        pvz = pad.vz;
    }

    EntryPoint here;
    here.x = px;
    here.y = py;
    here.z = pz;
    here.vx = pvx;
    here.vy = pvy;
    here.vz = pvz;
    here.mass = MASS_DRY + std::max(0.0, state[FUEL]);

    if (phase == EntryPhase::Coast) {
        check_age += dt;
        if (check_age >= 0.25) {
            check_age = 0.0;
            if (entry_should_flip(here, plant)) {
                phase = EntryPhase::Flip;
                std::cerr << "entry flip t=" << t
                          << " z=" << here.z
                          << " vz=" << here.vz
                          << " vh=" << std::hypot(here.vx, here.vy)
                          << "\n";
            }
        }
    }

    const double speed = std::hypot(here.vx, std::hypot(here.vy, here.vz));
    double ux = 0.0;
    double uy = 0.0;
    double uz = 1.0;
    if (phase == EntryPhase::Coast && speed > 1.0) {
        ux = here.vx / speed;
        uy = here.vy / speed;
        uz = here.vz / speed;
    } else if (phase != EntryPhase::Coast) {
        entry_brake_direction(here.vx, here.vy, here.vz, ux, uy, uz);
    }
    ask.attitude = quat_align_body_z(ux, uy, uz, quat_from_state(state));

    if (phase == EntryPhase::Flip) {
        double zx = 0.0;
        double zy = 0.0;
        double zz = 0.0;
        rotate_body_to_inertial(quat_from_state(state), 0.0, 0.0, 1.0, zx, zy, zz);
        const double aligned = zx * ux + zy * uy + zz * uz;
        const double cos_20 = std::cos(20.0 * 3.14159265358979323846 / 180.0);
        if (aligned >= cos_20) phase = EntryPhase::Brake;
    }

    if (phase == EntryPhase::Brake) {
        ask.fx = cfg.thrust_max * ux;
        ask.fy = cfg.thrust_max * uy;
        ask.fz = cfg.thrust_max * uz;
        if (entry_in_hop(here)) {
            phase = EntryPhase::Land;
            ask.guide = true;
            ask.force_solve = true;
            std::cerr << "entry handoff t=" << t
                      << " z=" << here.z
                      << " vz=" << here.vz
                      << " vh=" << std::hypot(here.vx, here.vy)
                      << " x=" << here.x
                      << " y=" << here.y
                      << " fuel=" << (here.mass - MASS_DRY)
                      << "\n";
        }
    }
    return ask;
}

}  // namespace icarus
