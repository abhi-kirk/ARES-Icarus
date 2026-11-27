// Quaternion attitude. The oracle for the rotation is the old ZYX matrix,
// written here so the plant and the check do not share one implementation.

#include "dynamics/air.h"
#include "dynamics/attitude.h"
#include "dynamics/constants.h"
#include "dynamics/rocket_6dof.h"
#include "gnc/control/gimbal_alloc.h"

#include <cmath>
#include <iostream>

namespace {

using namespace icarus;

int g_failed = 0;

void expect(bool ok, const char* name) {
    if (ok) {
        std::cout << "ok  " << name << "\n";
        return;
    }
    std::cerr << "FAIL " << name << "\n";
    g_failed += 1;
}

void expect_near(double got, double want, double tol, const char* name) {
    const bool ok = std::isfinite(got) && std::abs(got - want) <= tol;
    if (!ok) {
        std::cerr << "FAIL " << name << " got " << got << " want " << want << "\n";
        g_failed += 1;
        return;
    }
    std::cout << "ok  " << name << "\n";
}

void euler_matrix(
    double roll,
    double pitch,
    double yaw,
    double bx,
    double by,
    double bz,
    double& ix,
    double& iy,
    double& iz
) {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    const double r00 = cy * cp;
    const double r01 = cy * sp * sr - sy * cr;
    const double r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp;
    const double r11 = sy * sp * sr + cy * cr;
    const double r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp;
    const double r21 = cp * sr;
    const double r22 = cp * cr;

    ix = r00 * bx + r01 * by + r02 * bz;
    iy = r10 * bx + r11 * by + r12 * bz;
    iz = r20 * bx + r21 * by + r22 * bz;
}

bool close_vec(double ax, double ay, double az, double bx, double by, double bz, double tol) {
    return std::abs(ax - bx) <= tol && std::abs(ay - by) <= tol && std::abs(az - bz) <= tol;
}

bool rotation_matches(double roll, double pitch, double yaw) {
    const Quat q = quat_from_euler(roll, pitch, yaw);
    const double body[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
    };
    for (int i = 0; i < 3; ++i) {
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        rotate_body_to_inertial(q, body[i][0], body[i][1], body[i][2], ix, iy, iz);
        double ex = 0.0;
        double ey = 0.0;
        double ez = 0.0;
        euler_matrix(roll, pitch, yaw, body[i][0], body[i][1], body[i][2], ex, ey, ez);
        if (!close_vec(ix, iy, iz, ex, ey, ez, 1e-9)) return false;
    }
    return true;
}

State integrate(State state, double wx, double wy, double wz, double seconds) {
    state[WX] = wx;
    state[WY] = wy;
    state[WZ] = wz;
    ControlInput control = ControlInput::Zero(5);
    Disturbance wind = Disturbance::Zero(3);
    const double dt = 0.01;
    const int steps = static_cast<int>(std::lround(seconds / dt));
    double t = 0.0;
    for (int i = 0; i < steps; ++i) {
        SystemDynamics f = [&](double time, const State& s) {
            return rocket_dynamics_6dof(
                time, s, control, wind,
                IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT,
                0.0, GRAVITY
            );
        };
        state = Integrator::rk4_step(f, t, state, dt);
        normalize_state_quaternion(state);
        t += dt;
    }
    return state;
}

}  // namespace

int main() {
    const double deg = 3.14159265358979323846 / 180.0;

    {
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        rotate_body_to_inertial(quat_identity(), 0.0, 0.0, 1.0, ix, iy, iz);
        expect(close_vec(ix, iy, iz, 0.0, 0.0, 1.0, 1e-12), "identity keeps body z upright");
    }

    {
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        rotate_body_to_inertial(quat_from_euler(0.0, 90.0 * deg, 0.0), 0.0, 0.0, 1.0, ix, iy, iz);
        expect(close_vec(ix, iy, iz, 1.0, 0.0, 0.0, 1e-9), "pitch 90 lays body z on +x");
    }

    {
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        rotate_body_to_inertial(quat_from_euler(0.0, 0.0, 90.0 * deg), 1.0, 0.0, 0.0, ix, iy, iz);
        expect(close_vec(ix, iy, iz, 0.0, 1.0, 0.0, 1e-9), "yaw 90 turns body x onto +y");
    }

    {
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        rotate_body_to_inertial(quat_from_euler(90.0 * deg, 0.0, 0.0), 0.0, 1.0, 0.0, ix, iy, iz);
        expect(close_vec(ix, iy, iz, 0.0, 0.0, 1.0, 1e-9), "roll 90 turns body y onto +z");
    }

    const double rolls[] = {0.0, 20.0 * deg, -15.0 * deg};
    const double pitches[] = {0.0, 30.0 * deg, -40.0 * deg, 89.0 * deg, -89.0 * deg};
    const double yaws[] = {0.0, 25.0 * deg, -50.0 * deg};
    int matched = 0;
    int checked = 0;
    for (double roll : rolls) {
        for (double pitch : pitches) {
            for (double yaw : yaws) {
                checked += 1;
                if (rotation_matches(roll, pitch, yaw)) matched += 1;
            }
        }
    }
    expect(matched == checked, "quaternion matches the ZYX matrix on 45 attitudes");

    {
        const double roll = 12.0 * deg;
        const double pitch = -28.0 * deg;
        const double yaw = 40.0 * deg;
        double back_roll = 0.0;
        double back_pitch = 0.0;
        double back_yaw = 0.0;
        euler_from_quat(quat_from_euler(roll, pitch, yaw), back_roll, back_pitch, back_yaw);
        expect_near(back_roll, roll, 1e-9, "roll roundtrip");
        expect_near(back_pitch, pitch, 1e-9, "pitch roundtrip");
        expect_near(back_yaw, yaw, 1e-9, "yaw roundtrip");
    }

    {
        State state = State::Zero(STATE_SIZE);
        state[QW] = 1.0;
        state[POS_Z] = 100.0;
        state[FUEL] = 100.0;
        state = integrate(state, 0.0, 0.2, 0.0, 1.0);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        euler_from_quat(quat_from_state(state), roll, pitch, yaw);
        expect_near(pitch, 0.2, 1e-4, "a pitch rate of 0.2 rad/s for 1 s");
        expect_near(roll, 0.0, 1e-4, "roll stays zero on a pure pitch rate");
        const Quat q = quat_from_state(state);
        const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        expect_near(n, 1.0, 1e-9, "quaternion stays a unit after the pitch");
    }

    {
        State state = State::Zero(STATE_SIZE);
        quat_write(state, quat_from_euler(0.0, 80.0 * deg, 0.0));
        state[POS_Z] = 100.0;
        state[FUEL] = 100.0;
        state = integrate(state, 0.0, 0.5, 0.0, 1.0);
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        rotate_body_to_inertial(quat_from_state(state), 0.0, 0.0, 1.0, ix, iy, iz);
        const bool finite = std::isfinite(ix) && std::isfinite(iy) && std::isfinite(iz);
        expect(finite && iz < 0.0, "pitch passes through 90 deg and stays finite");
        const Quat q = quat_from_state(state);
        const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        expect_near(n, 1.0, 1e-8, "quaternion stays a unit through the old lock");
    }

    {
        const double roll = 8.0 * deg;
        const double pitch = -12.0 * deg;
        const double yaw = 6.0 * deg;
        double ex = 0.0;
        double ey = 0.0;
        double ez = 0.0;
        attitude_error_body(quat_from_euler(roll, 0.0, 0.0), quat_identity(), ex, ey, ez);
        expect_near(ex, -roll, 1e-9, "roll error is minus the angle");
        expect_near(ey, 0.0, 1e-9, "a pure roll has no pitch error");
        expect_near(ez, 0.0, 1e-9, "a pure roll has no yaw error");
        attitude_error_body(quat_from_euler(0.0, pitch, 0.0), quat_identity(), ex, ey, ez);
        expect_near(ey, -pitch, 1e-9, "pitch error is minus the angle");
        attitude_error_body(quat_from_euler(0.0, 0.0, yaw), quat_identity(), ex, ey, ez);
        expect_near(ez, -yaw, 1e-9, "yaw error is minus the angle");

        const double kp = 1.5 * IXX_DEFAULT;
        const double kd = 0.8 * IXX_DEFAULT;
        const double wx = 0.05;
        attitude_error_body(quat_from_euler(roll, 0.0, 0.0), quat_identity(), ex, ey, ez);
        const double tau_x = kp * ex - kd * wx;
        const double old_tau = -kp * roll - kd * wx;
        expect_near(tau_x, old_tau, 1e-6, "roll torque matches the old upright law");
    }

    {
        State state = State::Zero(STATE_SIZE);
        quat_write(state, quat_from_euler(0.0, 15.0 * deg, 10.0 * deg));
        state[POS_Z] = 100.0;
        state[FUEL] = 100.0;
        const double kp_xy = 1.5 * IXX_DEFAULT;
        const double kd_xy = 0.8 * IXX_DEFAULT;
        const double kp_z = 1.5 * IZZ_DEFAULT;
        const double kd_z = 0.8 * IZZ_DEFAULT;
        const Quat cmd = quat_identity();
        ControlInput control = ControlInput::Zero(6);
        Disturbance wind = Disturbance::Zero(3);
        const double dt = 0.01;
        double t = 0.0;
        for (int i = 0; i < 1200; ++i) {
            double ex = 0.0;
            double ey = 0.0;
            double ez = 0.0;
            attitude_error_body(quat_from_state(state), cmd, ex, ey, ez);
            control[3] = kp_xy * ex - kd_xy * state[WX];
            control[4] = kp_xy * ey - kd_xy * state[WY];
            control[5] = kp_z * ez - kd_z * state[WZ];
            SystemDynamics f = [&](double time, const State& s) {
                return rocket_dynamics_6dof(
                    time, s, control, wind,
                    IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT,
                    0.0, GRAVITY
                );
            };
            state = Integrator::rk4_step(f, t, state, dt);
            normalize_state_quaternion(state);
            t += dt;
        }
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        euler_from_quat(quat_from_state(state), roll, pitch, yaw);
        expect(std::abs(pitch) < 1.0 * deg, "upright command brings a 15 deg pitch home");
        expect(std::abs(yaw) < 1.0 * deg, "upright command brings a 10 deg yaw home");
        expect(std::abs(state[WY]) < 0.05, "pitch rate is quiet after the upright command");
        expect(std::abs(state[WZ]) < 0.05, "yaw rate is quiet after the upright command");
    }

    {
        double zx = 0.0;
        double zy = 0.0;
        double zz = 0.0;
        const Quat up = quat_align_body_z(0.0, 0.0, 1.0, quat_identity());
        rotate_body_to_inertial(up, 0.0, 0.0, 1.0, zx, zy, zz);
        expect(close_vec(zx, zy, zz, 0.0, 0.0, 1.0, 1e-9), "an upward force keeps body z upright");

        const double tilt = 10.0 * deg;
        const double ux = std::sin(tilt);
        const double uz = std::cos(tilt);
        const Quat leaned = quat_align_body_z(ux, 0.0, uz, quat_identity());
        rotate_body_to_inertial(leaned, 0.0, 0.0, 1.0, zx, zy, zz);
        expect(close_vec(zx, zy, zz, ux, 0.0, uz, 1e-9), "body z lies on a 10 deg force");
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        euler_from_quat(leaned, roll, pitch, yaw);
        expect_near(pitch, tilt, 1e-9, "a 10 deg force is a 10 deg pitch");
        expect_near(yaw, 0.0, 1e-9, "the pointing split does not add yaw");

        const Quat yawed = quat_from_euler(0.0, 0.0, 30.0 * deg);
        const Quat yawed_lean = quat_align_body_z(ux, 0.0, uz, yawed);
        rotate_body_to_inertial(yawed_lean, 0.0, 0.0, 1.0, zx, zy, zz);
        expect(close_vec(zx, zy, zz, ux, 0.0, uz, 1e-9), "a yawed body still lays z on the force");

        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        const double force = 10000.0;
        rotate_inertial_to_body(quat_identity(), force * ux, 0.0, force * uz, bx, by, bz);
        double thrust = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        allocate_gimbal(bx, by, bz, 15.0 * deg, thrust, dx, dy);
        expect_near(thrust, force, 1e-6, "residual split keeps the thrust length");
        expect_near(std::abs(dy), tilt, 1e-6, "an upright body gimbals the whole 10 deg");
        expect_near(dx, 0.0, 1e-6, "the 10 deg tilt is one gimbal axis");

        rotate_inertial_to_body(leaned, force * ux, 0.0, force * uz, bx, by, bz);
        allocate_gimbal(bx, by, bz, 15.0 * deg, thrust, dx, dy);
        expect_near(dx, 0.0, 1e-6, "once the body has the tilt, gimbal x is zero");
        expect_near(dy, 0.0, 1e-6, "once the body has the tilt, gimbal y is zero");
    }

    {
        const double pad = air_density_ratio(0.0, GRAVITY_EARTH);
        expect_near(pad, 1.0, 1e-12, "earth drag ratio is 1 on the pad");

        const double one_scale = air_density_ratio(AIR_SCALE_HEIGHT_M, GRAVITY_EARTH);
        expect_near(one_scale, std::exp(-1.0), 1e-12, "earth drag ratio is 1/e at 8.5 km");

        const double hop = air_density_ratio(300.0, GRAVITY_EARTH);
        expect_near(hop, std::exp(-300.0 / AIR_SCALE_HEIGHT_M), 1e-12, "a 300 m hop is slightly thinner air");

        const double below = air_density_ratio(-2.0, GRAVITY_EARTH);
        expect_near(below, 1.0, 1e-12, "below the pad the ratio stays at sea level");

        const double moon_pad = air_density_ratio(0.0, GRAVITY_MOON);
        const double moon_high = air_density_ratio(100000.0, GRAVITY_MOON);
        expect_near(moon_pad, 0.0, 0.0, "the moon has no air on the pad");
        expect_near(moon_high, 0.0, 0.0, "the moon has no air at 100 km");

        expect_near(air_density(0.0, GRAVITY_EARTH), SEA_LEVEL_DENSITY, 1e-12, "the pad density is 1.225 kg/m3");
        expect_near(
            air_density(300.0, GRAVITY_EARTH),
            SEA_LEVEL_DENSITY * std::exp(-300.0 / AIR_SCALE_HEIGHT_M),
            1e-12,
            "a 300 m hop is a little under 1.225 kg/m3"
        );
        expect_near(air_density(0.0, GRAVITY_MOON), 0.0, 0.0, "lunar density is zero on the pad");
    }

    {
        // 10 m/s of sideways speed, engine off, so the horizontal acceleration is drag.
        State state = State::Zero(STATE_SIZE);
        state[QW] = 1.0;
        state[VEL_X] = 10.0;
        ControlInput control = ControlInput::Zero(6);
        Disturbance wind = Disturbance::Zero(3);

        State sea = rocket_dynamics_6dof(
            0.0, state, control, wind,
            IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT,
            1.0, GRAVITY_EARTH
        );
        // drag = -0.5 * 1 * 10 * 10 = -50 N, mass = 1000 kg.
        expect_near(sea[VEL_X], -0.05, 1e-12, "sea-level drag matches the old law");

        state[POS_Z] = AIR_SCALE_HEIGHT_M;
        State high = rocket_dynamics_6dof(
            0.0, state, control, wind,
            IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT,
            1.0, GRAVITY_EARTH
        );
        expect_near(high[VEL_X], -0.05 / std::exp(1.0), 1e-12, "drag at one scale height is 1/e of sea level");

        state[POS_Z] = 0.0;
        State vacuum = rocket_dynamics_6dof(
            0.0, state, control, wind,
            IXX_DEFAULT, IYY_DEFAULT, IZZ_DEFAULT, GIMBAL_LEVER_DEFAULT,
            1.0, GRAVITY_MOON
        );
        expect_near(vacuum[VEL_X], 0.0, 1e-12, "lunar gravity zeros drag even with drag_scale 1");
        expect_near(vacuum[VEL_Z], -GRAVITY_MOON, 1e-12, "the moon still accelerates at 1.62");
    }

    if (g_failed > 0) {
        std::cerr << g_failed << " failed\n";
        return 1;
    }
    std::cout << "all passed\n";
    return 0;
}
