#include "sim/simulator.h"
#ifndef ICARUS_NO_RERUN
#include "sim/logger.h"
#endif
#include "dynamics/constants.h"
#include "dynamics/rocket_6dof.h"
#include "dynamics/state_indices.h"
#include "gnc/control/gimbal_alloc.h"
#include "gnc/control/thrust_lag.h"
#include "gnc/control/thrust_limits.h"

#include <algorithm>
#include <cmath>

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
    double gimbal_y
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
    sample.fx = fx;
    sample.fy = fy;
    sample.fz = fz;
    sample.pad_x = pad_x;
    sample.pad_y = pad_y;
    sample.roll = state[ROLL];
    sample.pitch = state[PITCH];
    sample.yaw = state[YAW];
    sample.gimbal_x = gimbal_x;
    sample.gimbal_y = gimbal_y;
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
    for (int i = 0; i < STATE_SIZE; ++i) {
        state[i] = prev_state[i] + alpha * (state[i] - prev_state[i]);
    }
    state[POS_Z] = 0.0;
}

void delivered_inertial_force(
    double T,
    double dx,
    double dy,
    double roll,
    double pitch,
    double yaw,
    double& Fx,
    double& Fy,
    double& Fz
) {
    const double Fx_b = T * std::sin(dy);
    const double Fy_b = T * (-std::sin(dx));
    const double Fz_b = T * std::cos(dx) * std::cos(dy);

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    Fx = (cy * cp) * Fx_b + (cy * sp * sr - sy * cr) * Fy_b + (cy * sp * cr + sy * sr) * Fz_b;
    Fy = (sy * cp) * Fx_b + (sy * sp * sr + cy * cr) * Fy_b + (sy * sp * cr - cy * sr) * Fz_b;
    Fz = (-sp) * Fx_b + (cp * sr) * Fy_b + (cp * cr) * Fz_b;
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

    // Lagged force command from the PID. On the 6-DOF plant this is the ask
    // that becomes thrust + gimbal; samples may show delivered force instead.
    double Fx_act = 0.0;
    double Fy_act = 0.0;
    double Fz_act = (MASS_DRY + state[FUEL]) * GRAVITY;

    double gimbal_x = 0.0;
    double gimbal_y = 0.0;
    const double gimbal_max_rad = cfg.gimbal_max_deg * (3.14159265358979323846 / 180.0);

    // Attitude PD gains are in N·m per rad / (rad/s). Scale with inertia so
    // a case that changes Ixx still gets a similar natural frequency.
    const double tau_kp = cfg.attitude_kp * cfg.Ixx;
    const double tau_kd = cfg.attitude_kd * cfg.Ixx;

    if (emit != nullptr) {
        Eigen::Vector2d pad0 = landing_pad.pad_position(t);
        (*emit)(sample_from(t, state, Fx_act, Fy_act, Fz_act, pad0[0], pad0[1], gimbal_x, gimbal_y));
    }

    while (state[POS_Z] > 0.0 && !should_stop(t, time_limit, stop)) {
        const State  prev_state = state;
        const double prev_t     = t;
        ++sim_counter;

        Eigen::Vector2d pad_pos = landing_pad.pad_position(t);
        Eigen::Vector2d pad_vel = landing_pad.pad_velocity();

        const double ref_vel_z = guidance.compute(state);
        const double feedforward = (MASS_DRY + state[FUEL]) * GRAVITY;
        double Fz_cmd = vel_ctrl.compute(state[VEL_Z], ref_vel_z, feedforward);

        if (sim_counter % inner_loop_divider == 0) {
            ref_vel_xy = guidance_h.compute(state, pad_pos, pad_vel);
            Fx_cmd = vel_ctrl_h_x.compute(state[VEL_X], ref_vel_xy[0]);
            Fy_cmd = vel_ctrl_h_y.compute(state[VEL_Y], ref_vel_xy[1]);
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

        Eigen::Vector3d wind_speeds = wind_model.wind_velocity(t);
        double Fx_log = Fx_act;
        double Fy_log = Fy_act;
        double Fz_log = Fz_act;

        if (cfg.use_6dof) {
            double T = 0.0;
            allocate_gimbal(Fx_act, Fy_act, Fz_act, gimbal_max_rad, T, gimbal_x, gimbal_y);

            // Gimbal side-force also makes a torque about the CG. Cancel that
            // in the RCS command so the stack can stay upright while the
            // gimbal still pushes sideways; the PD only fights leftover lean.
            const double Fx_b = T * std::sin(gimbal_y);
            const double Fy_b = T * (-std::sin(gimbal_x));
            double tau_x = 0.0;
            double tau_y = 0.0;
            attitude_rcs(
                state[ROLL], state[PITCH], state[WX], state[WY],
                tau_kp, tau_kd, tau_x, tau_y
            );
            tau_x += -cfg.gimbal_lever * Fy_b;
            tau_y += cfg.gimbal_lever * Fx_b;

            ControlInput control(5);
            control << T, gimbal_x, gimbal_y, tau_x, tau_y;

            SystemDynamics bound = [&](double time, const State& s) {
                return rocket_dynamics_6dof(
                    time, s, control, wind_speeds,
                    cfg.Ixx, cfg.Iyy, cfg.Izz, cfg.gimbal_lever
                );
            };
            state = Integrator::rk4_step(bound, t, state, dt_inner_v);

            delivered_inertial_force(
                T, gimbal_x, gimbal_y,
                prev_state[ROLL], prev_state[PITCH], prev_state[YAW],
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

        if (state[POS_Z] <= 0.0) {
            interpolate_touchdown(state, prev_state, t, prev_t, dt_inner_v);
        }

        state[FUEL] = std::max(0.0, state[FUEL]);

#ifndef ICARUS_NO_RERUN
        if (logger != nullptr) {
            logger->log(t, state, Fx_log, Fy_log, Fz_log, pad_pos[0], pad_pos[1], ref_vel_z, guidance.ref_vel_target());
        }
#else
        (void)logger;
#endif
        if (emit != nullptr) {
            Eigen::Vector2d pad_now = landing_pad.pad_position(t);
            (*emit)(sample_from(t, state, Fx_log, Fy_log, Fz_log, pad_now[0], pad_now[1], gimbal_x, gimbal_y));
        }
    }
}

}  // namespace icarus
