#include "sim/simulator.h"
#ifndef ICARUS_NO_RERUN
#include "sim/logger.h"
#endif
#include "dynamics/air.h"
#include "dynamics/attitude.h"
#include "dynamics/constants.h"
#include "dynamics/rocket_6dof.h"
#include "dynamics/state_indices.h"
#include "gnc/control/gimbal_alloc.h"
#include "gnc/control/thrust_lag.h"
#include "gnc/control/thrust_limits.h"
#include "gnc/guidance/ltv_mpc.h"
#include "sim/flight_phase.h"
#include "gnc/guidance/scvx_guidance.h"
#include "gnc/guidance/soft_land_reference.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace icarus {

namespace {

FlightSample sample_from(
    double t,
    const State& state,
    double fx,
    double fy,
    double fz,
    double pad_x,
    double pad_y,
    double gimbal_x,
    double gimbal_y,
    const MpcSolveResult* plan,
    double gravity
) {
    FlightSample sample;
    sample.t = t;
    sample.x = state[POS_X];
    sample.y = state[POS_Y];
    sample.z = state[POS_Z];
    sample.vx = state[VEL_X];
    sample.vy = state[VEL_Y];
    sample.vz = state[VEL_Z];
    sample.fuel = state[FUEL];
    sample.rho = air_density(state[POS_Z], gravity);
    sample.fx = fx;
    sample.fy = fy;
    sample.fz = fz;
    sample.pad_x = pad_x;
    sample.pad_y = pad_y;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    euler_from_quat(quat_from_state(state), roll, pitch, yaw);
    sample.roll = roll;
    sample.pitch = pitch;
    sample.yaw = yaw;
    sample.gimbal_x = gimbal_x;
    sample.gimbal_y = gimbal_y;
    if (plan != nullptr && !plan->plan_pos.empty()) {
        // plan_pos[0] is "now"; use the first future knot for the marker.
        const size_t idx = plan->plan_pos.size() > 1 ? 1 : 0;
        sample.plan_x = plan->plan_pos[idx].x();
        sample.plan_y = plan->plan_pos[idx].y();
        sample.plan_z = plan->plan_pos[idx].z();
        sample.plan_valid = true;
        sample.plan_trail.clear();
        for (size_t i = idx; i < plan->plan_pos.size(); ++i) {
            sample.plan_trail.push_back(plan->plan_pos[i].x());
            sample.plan_trail.push_back(plan->plan_pos[i].y());
            sample.plan_trail.push_back(plan->plan_pos[i].z());
        }
    }
    return sample;
}

bool should_stop(double t, double time_limit, const std::atomic<bool>* stop) {
    if (stop != nullptr && stop->load()) return true;
    if (time_limit > 0.0 && t >= time_limit) return true;
    return false;
}

void interpolate_touchdown(State& state, const State& prev_state, double& t, double prev_t, double dt) {
    const double y0 = prev_state[POS_Z];
    const double y1 = state[POS_Z];
    const double denom = y0 - y1;
    double alpha = 1.0;
    if (denom > 0.0) alpha = std::clamp(y0 / denom, 0.0, 1.0);

    t = prev_t + alpha * dt;
    same_quaternion_hemisphere(prev_state, state);
    for (int i = 0; i < STATE_SIZE; ++i) {
        state[i] = prev_state[i] + alpha * (state[i] - prev_state[i]);
    }
    state[POS_Z] = 0.0;
    normalize_state_quaternion(state);
}

void delivered_inertial_force(
    double T,
    double dx,
    double dy,
    const State& at,
    double& Fx,
    double& Fy,
    double& Fz
) {
    const double Fx_b = T * std::sin(dy);
    const double Fy_b = T * (-std::sin(dx));
    const double Fz_b = T * std::cos(dx) * std::cos(dy);
    rotate_body_to_inertial(quat_from_state(at), Fx_b, Fy_b, Fz_b, Fx, Fy, Fz);
}

}  // namespace

void Simulator::run(
    State& state,
    double t_start,
    const SimConfig& cfg,
    CascadeGuidance& guidance,
    HorizontalGuidance& guidance_h,
    Controller& vel_ctrl,
    Controller& vel_ctrl_h_x,
    Controller& vel_ctrl_h_y,
    const DynamicsFunction& dynamics,
    const WindModel& wind_model,
    const LandingPad& landing_pad,
    Logger* logger,
    const std::atomic<bool>* stop,
    const SampleEmit* emit,
    double time_limit
) {
    double t  = t_start;
    double dt_inner_v = cfg.timestep_inner;
    double dt_inner_h = cfg.timestep_inner_h;

    double ratio = dt_inner_h / dt_inner_v;
    int inner_loop_divider = std::max(1, static_cast<int>(std::lround(ratio)));
    int sim_counter = 0;

    Eigen::Vector2d ref_vel_xy = Eigen::Vector2d::Zero();
    double Fx_cmd = vel_ctrl_h_x.compute(state[VEL_X], 0.0);
    double Fy_cmd = vel_ctrl_h_y.compute(state[VEL_Y], 0.0);
    const double gravity = cfg.gravity;
    double Fz_cmd = (MASS_DRY + state[FUEL]) * gravity;

    // Lagged force command from the PID / MPC. On the 6-DOF plant this is the
    // ask that becomes thrust + gimbal; samples may show delivered force instead.
    double Fx_act = 0.0;
    double Fy_act = 0.0;
    double Fz_act = Fz_cmd;

    double gimbal_x = 0.0;
    double gimbal_y = 0.0;
    const double gimbal_max_rad = cfg.gimbal_max_deg * (3.14159265358979323846 / 180.0);

    // Attitude PD gains are in N·m per rad / (rad/s). Scale with inertia so
    // a case that changes Ixx still gets a similar natural frequency.
    // Roll and pitch share the transverse inertia. Yaw uses Izz.
    const double tau_kp = cfg.attitude_kp * cfg.Ixx;
    const double tau_kd = cfg.attitude_kd * cfg.Ixx;
    const double tau_kp_z = cfg.attitude_kp * cfg.Izz;
    const double tau_kd_z = cfg.attitude_kd * cfg.Izz;
    // Holds the last pointing command. With the engine off this stays put
    // and the jets track it. With the engine on it becomes the force direction.
    Quat attitude_cmd = quat_identity();

    const bool use_mpc = (cfg.controller == "mpc");
    const bool use_scvx = (cfg.controller == "scvx");
    SoftLandReference soft_ref;
    LtvMpc mpc;
    MpcSolveResult last_plan;
    int mpc_divider = 1;
    if (use_mpc) {
        Eigen::Vector2d pad0 = landing_pad.pad_position(t);
        const double mass0 = MASS_DRY + state[FUEL];
        // Rebuild the soft-land reference from the live state at every MPC
        // tick. A once-built open-loop profile goes stale under wind and then
        // commands a post-skim climb (MC hard landings at |vz| >> 2 m/s).
        soft_ref.build(
            t,
            Eigen::Vector3d(state[POS_X], state[POS_Y], state[POS_Z]),
            Eigen::Vector3d(state[VEL_X], state[VEL_Y], state[VEL_Z]),
            mass0,
            cfg.thrust_min,
            cfg.thrust_max,
            cfg.pointing_cone_deg,
            cfg.mpc_dt,
            pad0[0],
            pad0[1],
            gravity
        );
        mpc.configure(
            cfg.mpc_horizon,
            cfg.mpc_dt,
            cfg.mpc_w_pos,
            cfg.mpc_w_vel,
            cfg.mpc_w_u,
            cfg.mpc_w_terminal,
            cfg.thrust_min,
            cfg.thrust_max,
            cfg.pointing_cone_deg,
            cfg.glide_cone_deg,
            cfg.drag_scale,
            gravity
        );
        mpc.set_reference(&soft_ref);
        mpc_divider = std::max(1, static_cast<int>(std::lround(cfg.mpc_dt / dt_inner_v)));

        // An entry coast is not a landing problem. The first solve waits
        // until the brake has put the rocket on a hop these laws already fly.
        if (!cfg.entry) {
            Eigen::Vector3d wind0 = wind_model.wind_velocity(t);
            last_plan = mpc.solve(
                t,
                Eigen::Vector3d(state[POS_X], state[POS_Y], state[POS_Z]),
                Eigen::Vector3d(state[VEL_X], state[VEL_Y], state[VEL_Z]),
                mass0,
                pad0[0],
                pad0[1],
                wind0.x(),
                wind0.y()
            );
            Fx_cmd = last_plan.force.x();
            Fy_cmd = last_plan.force.y();
            Fz_cmd = last_plan.force.z();
            Fx_act = Fx_cmd;
            Fy_act = Fy_cmd;
            Fz_act = Fz_cmd;
        }
    }

    ScvxGuidance scvx;
    bool scvx_refined = false;
    bool scvx_late = false;
    if (use_scvx) {
        scvx.configure(
            cfg.thrust_min,
            cfg.thrust_max,
            cfg.pointing_cone_deg,
            cfg.gimbal_max_deg,
            cfg.glide_cone_deg,
            cfg.drag_scale,
            gravity
        );
        if (!cfg.entry) {
            Eigen::Vector2d pad0 = landing_pad.pad_position(t);
            Eigen::Vector3d wind0 = wind_model.wind_velocity(t);
            last_plan = scvx.solve(
                t,
                Eigen::Vector3d(state[POS_X], state[POS_Y], state[POS_Z]),
                Eigen::Vector3d(state[VEL_X], state[VEL_Y], state[VEL_Z]),
                MASS_DRY + state[FUEL],
                pad0[0],
                pad0[1],
                wind0.x(),
                wind0.y()
            );
            const Eigen::Vector3d f0 = scvx.force_at(t);
            Fx_cmd = f0.x();
            Fy_cmd = f0.y();
            Fz_cmd = f0.z();
            Fx_act = Fx_cmd;
            Fy_act = Fy_cmd;
            Fz_act = Fz_cmd;
        }
        // The first plan is the divert. One more look happens once the
        // rocket is low. Looking every few seconds was starting the divert
        // over, so the tank ran dry before the rocket was down.
    }

    EntryPhase entry_phase = EntryPhase::Land;
    double entry_check_age = 0.0;
    if (cfg.entry) {
        entry_phase = EntryPhase::Coast;
        const double speed0 = std::hypot(state[VEL_X], std::hypot(state[VEL_Y], state[VEL_Z]));
        if (speed0 > 1.0) {
            const Quat nose = quat_align_body_z(
                state[VEL_X] / speed0,
                state[VEL_Y] / speed0,
                state[VEL_Z] / speed0,
                quat_identity()
            );
            quat_write(state, nose);
            attitude_cmd = nose;
        }
        Fx_cmd = 0.0;
        Fy_cmd = 0.0;
        Fz_cmd = 0.0;
        Fx_act = 0.0;
        Fy_act = 0.0;
        Fz_act = 0.0;
    }

    if (emit != nullptr) {
        Eigen::Vector2d pad0 = landing_pad.pad_position(t);
        (*emit)(sample_from(
            t, state, Fx_act, Fy_act, Fz_act, pad0[0], pad0[1], gimbal_x, gimbal_y,
            (use_mpc || use_scvx) ? &last_plan : nullptr, cfg.gravity
        ));
    }

    while (state[POS_Z] > 0.0 && !should_stop(t, time_limit, stop)) {
        const State  prev_state = state;
        const double prev_t     = t;
        ++sim_counter;

        Eigen::Vector2d pad_pos = landing_pad.pad_position(t);
        Eigen::Vector2d pad_vel = landing_pad.pad_velocity();

        double ref_vel_z = 0.0;
        bool guide = true;
        bool force_solve = false;
        if (cfg.entry && entry_phase != EntryPhase::Land) {
            const EntryAsk ask = update_entry(
                entry_phase, entry_check_age, dt_inner_v, t, state, cfg, attitude_cmd
            );
            guide = ask.guide;
            force_solve = ask.force_solve;
            if (!guide) {
                Fx_cmd = ask.fx;
                Fy_cmd = ask.fy;
                Fz_cmd = ask.fz;
                attitude_cmd = ask.attitude;
            }
        }
        if (guide && use_mpc) {
            if (force_solve || sim_counter % mpc_divider == 0) {
                const double mass = MASS_DRY + state[FUEL];
                soft_ref.build(
                    t,
                    Eigen::Vector3d(state[POS_X], state[POS_Y], state[POS_Z]),
                    Eigen::Vector3d(state[VEL_X], state[VEL_Y], state[VEL_Z]),
                    mass,
                    cfg.thrust_min,
                    cfg.thrust_max,
                    cfg.pointing_cone_deg,
                    cfg.mpc_dt,
                    pad_pos[0],
                    pad_pos[1],
                    gravity
                );
                Eigen::Vector3d wind_now = wind_model.wind_velocity(t);
                last_plan = mpc.solve(
                    t,
                    Eigen::Vector3d(state[POS_X], state[POS_Y], state[POS_Z]),
                    Eigen::Vector3d(state[VEL_X], state[VEL_Y], state[VEL_Z]),
                    mass,
                    pad_pos[0],
                    pad_pos[1],
                    wind_now.x(),
                    wind_now.y()
                );
                Fx_cmd = last_plan.force.x();
                Fy_cmd = last_plan.force.y();
                Fz_cmd = last_plan.force.z();
            }
        } else if (guide && use_scvx) {
            // The point-mass plan arrives slower than the rocket, and on a
            // long lunar burn it is still sliding sideways when the plan
            // goes vertical. One new plan once the divert is mostly done,
            // and one more once the rocket is low, take that speed off.
            // A plan that does not land leaves the burn we already have.
            const bool mid_update = !scvx_refined && state[POS_Z] < 40.0 && state[POS_Z] > 8.0;
            const bool late_update = scvx_refined && !scvx_late && state[POS_Z] < 12.0 && state[POS_Z] > 3.0;
            if (force_solve || mid_update || late_update) {
                if (mid_update) scvx_refined = true;
                if (late_update) scvx_late = true;
                const double mass = MASS_DRY + state[FUEL];
                Eigen::Vector3d wind_now = wind_model.wind_velocity(t);
                last_plan = scvx.solve(
                    t,
                    Eigen::Vector3d(state[POS_X], state[POS_Y], state[POS_Z]),
                    Eigen::Vector3d(state[VEL_X], state[VEL_Y], state[VEL_Z]),
                    mass,
                    pad_pos[0],
                    pad_pos[1],
                    wind_now.x(),
                    wind_now.y()
                );
            }
            const Eigen::Vector3d asked = scvx.force_at(t);
            Fx_cmd = asked.x();
            Fy_cmd = asked.y();
            Fz_cmd = asked.z();
            // Already slow and within a few metres. A last hard brake here
            // climbs the rocket back up, and the sink that follows burns
            // fuel the tracking planner does not spend. Keep a mild descent
            // so the hop finishes in the landing window.
            if (state[POS_Z] < 3.0 && state[VEL_Z] > -1.6 && state[VEL_Z] < 0.4) {
                const double mass_now = MASS_DRY + state[FUEL];
                const double az = Fz_cmd / std::max(mass_now, 1.0) - gravity;
                if (az > -0.4 && Fz_cmd > 1.0) {
                    const double fz_hold = std::max(cfg.thrust_min, mass_now * (gravity - 0.4));
                    if (fz_hold < Fz_cmd) {
                        const double scale = fz_hold / Fz_cmd;
                        Fx_cmd *= scale;
                        Fy_cmd *= scale;
                        Fz_cmd = fz_hold;
                    }
                }
            }
        } else if (guide) {
            ref_vel_z = guidance.compute(state);
            const double feedforward = (MASS_DRY + state[FUEL]) * gravity;
            Fz_cmd = vel_ctrl.compute(state[VEL_Z], ref_vel_z, feedforward);

            if (sim_counter % inner_loop_divider == 0) {
                ref_vel_xy = guidance_h.compute(state, pad_pos, pad_vel);
                Fx_cmd = vel_ctrl_h_x.compute(state[VEL_X], ref_vel_xy[0]);
                Fy_cmd = vel_ctrl_h_y.compute(state[VEL_Y], ref_vel_xy[1]);
            }
            prefer_vertical_thrust(
                Fx_cmd, Fy_cmd, Fz_cmd,
                cfg.thrust_max,
                cfg.pointing_cone_deg
            );
        }

        apply_thrust_limits(
            Fx_cmd, Fy_cmd, Fz_cmd,
            cfg.thrust_min,
            cfg.thrust_max,
            cfg.pointing_cone_deg
        );
        apply_thrust_lag(
            Fx_act, Fy_act, Fz_act,
            Fx_cmd, Fy_cmd, Fz_cmd,
            dt_inner_v,
            cfg.thrust_tau,
            cfg.thrust_slew
        );

        // Dynamics also drop thrust at empty, but the gimbal torque cancel
        // is computed here, so the command has to die here too.
        if (state[FUEL] <= 0.0) {
            Fx_act = 0.0;
            Fy_act = 0.0;
            Fz_act = 0.0;
        }

        // Coast and the flip keep the engine off. A thrust floor would
        // otherwise light it the moment the command is empty.
        if (cfg.entry && (entry_phase == EntryPhase::Coast || entry_phase == EntryPhase::Flip)) {
            Fx_cmd = 0.0;
            Fy_cmd = 0.0;
            Fz_cmd = 0.0;
            Fx_act = 0.0;
            Fy_act = 0.0;
            Fz_act = 0.0;
        }

        Eigen::Vector3d wind_speeds = wind_model.wind_velocity(t);
        double Fx_log = Fx_act;
        double Fy_log = Fy_act;
        double Fz_log = Fz_act;

        if (cfg.use_6dof) {
            const Quat q_now = quat_normalize(quat_from_state(state));
            const double force_len = std::sqrt(Fx_act * Fx_act + Fy_act * Fy_act + Fz_act * Fz_act);
            // A newton is enough to define a direction. Below that the jets
            // keep the command they already have, which starts upright.
            if (force_len > 1.0) {
                attitude_cmd = quat_align_body_z(
                    Fx_act / force_len,
                    Fy_act / force_len,
                    Fz_act / force_len,
                    q_now
                );
            }

            double Fx_b_des = 0.0;
            double Fy_b_des = 0.0;
            double Fz_b_des = 0.0;
            rotate_inertial_to_body(q_now, Fx_act, Fy_act, Fz_act, Fx_b_des, Fy_b_des, Fz_b_des);
            double T = 0.0;
            allocate_gimbal(Fx_b_des, Fy_b_des, Fz_b_des, gimbal_max_rad, T, gimbal_x, gimbal_y);

            // The gimbal still makes a moment about the CG. Cancel it here so
            // the jet PD tracks only the pointing error.
            const double Fx_b = T * std::sin(gimbal_y);
            const double Fy_b = T * (-std::sin(gimbal_x));
            double ex = 0.0;
            double ey = 0.0;
            double ez = 0.0;
            attitude_error_body(quat_from_state(state), attitude_cmd, ex, ey, ez);
            double tau_x = tau_kp * ex - tau_kd * state[WX];
            double tau_y = tau_kp * ey - tau_kd * state[WY];
            double tau_z = tau_kp_z * ez - tau_kd_z * state[WZ];
            tau_x += -cfg.gimbal_lever * Fy_b;
            tau_y += cfg.gimbal_lever * Fx_b;

            ControlInput control(6);
            control << T, gimbal_x, gimbal_y, tau_x, tau_y, tau_z;

            SystemDynamics bound = [&](double time, const State& s) {
                return rocket_dynamics_6dof(
                    time, s, control, wind_speeds,
                    cfg.Ixx, cfg.Iyy, cfg.Izz, cfg.gimbal_lever,
                    cfg.drag_scale, gravity, cfg.exhaust_speed, cfg.planet_centered
                );
            };
            state = Integrator::rk4_step(bound, t, state, dt_inner_v);
            normalize_state_quaternion(state);

            delivered_inertial_force(
                T, gimbal_x, gimbal_y,
                prev_state,
                Fx_log, Fy_log, Fz_log
            );
            (void)dynamics;
        } else {
            ControlInput control(3);
            control << Fx_act, Fy_act, Fz_act;
            SystemDynamics bound = [&](double time, const State& s) {
                return dynamics(time, s, control, wind_speeds);
            };
            state = Integrator::rk4_step(bound, t, state, dt_inner_v);
            gimbal_x = 0.0;
            gimbal_y = 0.0;
        }

        t += dt_inner_v;

        // Soft contact: near the pad with little vertical speed counts as
        // landed. Miss gate stays tight so a 20 m skim is not scored a success.
        const double miss_pad = std::hypot(
            state[POS_X] - pad_pos[0],
            state[POS_Y] - pad_pos[1]
        );
        const bool soft_contact =
            state[POS_Z] > 0.0 &&
            state[POS_Z] < 2.0 &&
            state[VEL_Z] > -2.0 &&
            state[VEL_Z] < 0.5 &&
            miss_pad < 5.0;
        if (state[POS_Z] <= 0.0 || soft_contact) {
            if (state[POS_Z] > 0.0) {
                state[POS_Z] = 0.0;
                state[VEL_Z] = std::min(0.0, state[VEL_Z]);
            } else {
                interpolate_touchdown(state, prev_state, t, prev_t, dt_inner_v);
            }
        }

        state[FUEL] = std::max(0.0, state[FUEL]);

#ifndef ICARUS_NO_RERUN
        if (logger != nullptr) {
            logger->log(t, state, Fx_log, Fy_log, Fz_log, pad_pos[0], pad_pos[1], ref_vel_z, guidance.ref_vel_target());
        }
#else
        (void)logger;
        (void)ref_vel_z;
#endif
        if (emit != nullptr) {
            Eigen::Vector2d pad_now = landing_pad.pad_position(t);
            (*emit)(sample_from(
                t, state, Fx_log, Fy_log, Fz_log, pad_now[0], pad_now[1], gimbal_x, gimbal_y,
                (use_mpc || use_scvx) ? &last_plan : nullptr, cfg.gravity
            ));
        }
    }
}

}  // namespace icarus
