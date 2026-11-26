// Quaternion attitude. The oracle for the rotation is the old ZYX matrix,
// written here so the plant and the check do not share one implementation.

#include "dynamics/attitude.h"
#include "dynamics/constants.h"
#include "dynamics/rocket_6dof.h"

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

    if (g_failed > 0) {
        std::cerr << g_failed << " failed\n";
        return 1;
    }
    std::cout << "all passed\n";
    return 0;
}
