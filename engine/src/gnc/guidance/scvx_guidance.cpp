#include "gnc/guidance/scvx_guidance.h"

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <osqp.h>

namespace icarus {

namespace {

const double kVe = MAX_THRUST / MAX_FLOW_RATE;
const double kDt = 0.5;
const int kPasses = 5;
const double kInnerBall = 0.90;

csc* owned_csc_from_triplets(
    int rows,
    int cols,
    const std::vector<c_int>& row_idx,
    const std::vector<c_float>& values,
    const std::vector<c_int>& col_ptr
) {
    const c_int nnz = static_cast<c_int>(values.size());
    csc* M = csc_spalloc(rows, cols, nnz, 1, 0);
    if (M == nullptr) return nullptr;
    for (c_int k = 0; k < nnz; ++k) {
        M->i[k] = row_idx[static_cast<size_t>(k)];
        M->x[k] = values[static_cast<size_t>(k)];
    }
    for (int c = 0; c <= cols; ++c) {
        M->p[c] = col_ptr[static_cast<size_t>(c)];
    }
    return M;
}

csc* dense_to_csc_upper(const Eigen::MatrixXd& dense) {
    const int n = static_cast<int>(dense.rows());
    std::vector<c_int> row_idx;
    std::vector<c_float> values;
    std::vector<c_int> col_ptr(static_cast<size_t>(n + 1), 0);
    for (int col = 0; col < n; ++col) {
        col_ptr[static_cast<size_t>(col)] = static_cast<c_int>(values.size());
        for (int row = 0; row <= col; ++row) {
            const double v = dense(row, col);
            if (std::abs(v) < 1e-16) continue;
            row_idx.push_back(row);
            values.push_back(v);
        }
    }
    col_ptr[static_cast<size_t>(n)] = static_cast<c_int>(values.size());
    return owned_csc_from_triplets(n, n, row_idx, values, col_ptr);
}

csc* dense_to_csc(const Eigen::MatrixXd& dense) {
    const int rows = static_cast<int>(dense.rows());
    const int cols = static_cast<int>(dense.cols());
    std::vector<c_int> row_idx;
    std::vector<c_float> values;
    std::vector<c_int> col_ptr(static_cast<size_t>(cols + 1), 0);
    for (int col = 0; col < cols; ++col) {
        col_ptr[static_cast<size_t>(col)] = static_cast<c_int>(values.size());
        for (int row = 0; row < rows; ++row) {
            const double v = dense(row, col);
            if (std::abs(v) < 1e-16) continue;
            row_idx.push_back(row);
            values.push_back(v);
        }
    }
    col_ptr[static_cast<size_t>(cols)] = static_cast<c_int>(values.size());
    return owned_csc_from_triplets(rows, cols, row_idx, values, col_ptr);
}

std::vector<Eigen::Vector3d> ball_directions() {
    std::vector<Eigen::Vector3d> dirs;
    dirs.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
    const double elev[2] = {20.0 * M_PI / 180.0, 45.0 * M_PI / 180.0};
    for (double el : elev) {
        for (int i = 0; i < 8; ++i) {
            const double az = (2.0 * M_PI * i) / 8.0;
            const double s = std::sin(el);
            dirs.push_back(Eigen::Vector3d(
                s * std::cos(az),
                s * std::sin(az),
                std::cos(el)
            ));
        }
    }
    return dirs;
}

Eigen::Vector3d drag_accel(
    const Eigen::Vector3d& vel,
    double wind_x,
    double wind_y,
    double mass,
    double drag_scale
) {
    const double vx = vel.x() - wind_x;
    const double vy = vel.y() - wind_y;
    const double vz = vel.z();
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double coeff = 0.5 * std::max(0.0, drag_scale);
    const double m = std::max(mass, 1.0);
    return Eigen::Vector3d(
        -coeff * vx * speed / m,
        -coeff * vy * speed / m,
        -coeff * vz * speed / m
    );
}

// Same limits the plant applies, so the fuel we score is fuel the rocket burns.
Eigen::Vector3d clip_force(
    Eigen::Vector3d force,
    double thrust_min,
    double thrust_max,
    double tan_point
) {
    if (force.z() < 0.0) force.z() = 0.0;
    if (tan_point < 1.0e5) {
        const double fh = std::hypot(force.x(), force.y());
        const double cap = force.z() * tan_point;
        if (fh > cap && fh > 1e-12) {
            const double scale = cap / fh;
            force.x() *= scale;
            force.y() *= scale;
        }
    }
    const double mag = force.norm();
    const double max_n = std::max(thrust_min, thrust_max);
    if (mag > max_n && mag > 1e-12) {
        force *= max_n / mag;
    } else if (mag < thrust_min && mag > 1e-12) {
        force *= thrust_min / mag;
    } else if (mag <= 1e-12 && thrust_min > 0.0) {
        force = Eigen::Vector3d(0.0, 0.0, thrust_min);
    }
    return force;
}

struct Rollout {
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Vector3d> vel;
    double fuel_kg = 0.0;
    double miss = 1.0e9;
    double vz = 0.0;
    bool soft = false;
};

Rollout rollout_forces(
    const std::vector<Eigen::Vector3d>& force,
    const Eigen::Vector3d& pos0,
    const Eigen::Vector3d& vel0,
    double mass0,
    double pad_x,
    double pad_y,
    double wind_x,
    double wind_y,
    double drag_scale,
    double thrust_min,
    double thrust_max,
    double tan_point,
    double dt
) {
    Rollout out;
    Eigen::Vector3d pos = pos0;
    Eigen::Vector3d vel = vel0;
    double mass = std::max(mass0, MASS_DRY);
    const double fuel0 = std::max(0.0, mass0 - MASS_DRY);
    out.pos.push_back(pos);
    out.vel.push_back(vel);

    bool scored = false;
    for (const Eigen::Vector3d& raw : force) {
        Eigen::Vector3d u = clip_force(raw, thrust_min, thrust_max, tan_point);
        if (mass <= MASS_DRY + 1e-6) {
            u.setZero();
        }
        const Eigen::Vector3d a = u / mass
            + Eigen::Vector3d(0.0, 0.0, -GRAVITY)
            + drag_accel(vel, wind_x, wind_y, mass, drag_scale);
        pos += dt * vel + 0.5 * dt * dt * a;
        vel += dt * a;
        const double used = (u.norm() * dt) / kVe;
        mass = std::max(MASS_DRY, mass - used);
        out.fuel_kg += used;
        out.pos.push_back(pos);
        out.vel.push_back(vel);
        if (!scored && pos.z() <= 0.5) {
            scored = true;
            out.fuel_kg = std::min(out.fuel_kg, fuel0);
            out.miss = std::hypot(pos.x() - pad_x, pos.y() - pad_y);
            out.vz = vel.z();
            // A little slower and closer than the flight gate, so the real
            // rocket still counts as a soft landing after it follows the plan.
            out.soft = (out.miss <= 15.0 && std::abs(out.vz) <= 1.6 && pos.z() > -1.0);
            break;
        }
    }
    if (!scored) {
        out.fuel_kg = std::min(out.fuel_kg, fuel0);
        out.miss = std::hypot(pos.x() - pad_x, pos.y() - pad_y);
        out.vz = vel.z();
        out.soft = (pos.z() <= 1.0 && out.miss <= 15.0 && std::abs(out.vz) <= 1.6);
    }
    return out;
}

// Lower is better. A soft landing sorts by fuel. A slow stop still above the
// pad beats a fast impact, because a short sink can finish the slow one.
double rollout_quality(const Rollout& flew, double tf) {
    if (flew.soft) {
        // A long horizon that only grazes the pad halfway through is a
        // different burn after that graze. The real rocket is late, so it
        // flies that later burn and leaves the pad. Prefer a plan that
        // reaches the ground near the end of its own horizon.
        const double touch_t = (static_cast<double>(flew.pos.size()) - 1.0) * kDt;
        const double early = (tf > 0.0 && touch_t < tf - 2.0) ? 2000.0 : 0.0;
        return flew.fuel_kg + early;
    }
    const double z = flew.pos.empty() ? 1.0e9 : flew.pos.back().z();
    const double speed = flew.vel.empty() ? 1.0e9 : flew.vel.back().norm();
    if (z > 0.5 && z <= 40.0 && flew.miss <= 15.0 && speed <= 6.0) {
        return 1000.0 + z + speed;
    }
    return 10000.0 + 10.0 * std::abs(flew.vz) + flew.miss;
}

std::vector<double> mass_along(
    const std::vector<Eigen::Vector3d>& force,
    double mass0,
    double dt
) {
    std::vector<double> mass(force.size(), mass0);
    double m = std::max(mass0, MASS_DRY);
    for (size_t k = 0; k < force.size(); ++k) {
        mass[k] = std::max(m, MASS_DRY);
        const double used = force[k].norm() * dt / kVe;
        m = std::max(MASS_DRY, m - used);
    }
    return mass;
}

struct QpStep {
    bool ok = false;
    std::vector<Eigen::Vector3d> force;
};

QpStep solve_qp(
    const std::vector<Eigen::Vector3d>& u_ref,
    const std::vector<Eigen::Vector3d>& v_ref,
    const Eigen::Vector3d& pos0,
    const Eigen::Vector3d& vel0,
    double mass0,
    double pad_x,
    double pad_y,
    double wind_x,
    double wind_y,
    double drag_scale,
    double thrust_min,
    double thrust_max,
    double tan_point,
    double tan_glide,
    double rho,
    double dt
) {
    QpStep step;
    const int N = static_cast<int>(u_ref.size());
    if (N < 2) return step;

    const auto dirs = ball_directions();
    const int n_dir = static_cast<int>(dirs.size());
    const bool limit_tilt = tan_point < 1.0e5;
    const bool limit_glide = tan_glide < 1.0e5;
    const double fuel_avail = std::max(0.0, mass0 - MASS_DRY);

    const int nU = N * 3;
    const int nSig = N;
    const int nG = limit_glide ? N * 4 : 0;
    const int n_var = nU + nSig + nG;
    const double kScale = 1000.0;
    auto iu = [&](int k, int i) { return k * 3 + i; };
    auto isig = [&](int k) { return nU + k; };
    auto ig = [&](int s) { return nU + nSig + s; };

    // State after each step is an exact straight-line function of the thrust
    // list. The solver cannot claim a soft landing the thrust does not fly.
    const std::vector<double> mass = mass_along(u_ref, mass0, dt);
    Eigen::MatrixXd A_step = Eigen::MatrixXd::Identity(6, 6);
    A_step.block(0, 3, 3, 3) = dt * Eigen::Matrix3d::Identity();

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, nU);
    Eigen::VectorXd aff(6);
    aff << pos0.x() - pad_x, pos0.y() - pad_y, pos0.z(), vel0.x(), vel0.y(), vel0.z();

    std::vector<Eigen::MatrixXd> H_at(static_cast<size_t>(N));
    std::vector<Eigen::VectorXd> aff_at(static_cast<size_t>(N));
    for (int k = 0; k < N; ++k) {
        const double m = std::max(mass[static_cast<size_t>(k)], 1.0);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
        const double c_r = 0.5 * dt * dt / m;
        const double c_v = dt / m;
        B.block(0, 0, 3, 3) = (c_r * kScale) * Eigen::Matrix3d::Identity();
        B.block(3, 0, 3, 3) = (c_v * kScale) * Eigen::Matrix3d::Identity();

        Eigen::Vector3d offset(0.0, 0.0, -GRAVITY);
        if (drag_scale > 0.0) {
            offset += drag_accel(
                v_ref[static_cast<size_t>(k)], wind_x, wind_y, m, drag_scale
            );
        }
        Eigen::VectorXd c(6);
        c.head(3) = 0.5 * dt * dt * offset;
        c.tail(3) = dt * offset;

        aff = A_step * aff + c;
        H = A_step * H;
        H.block(0, k * 3, 6, 3) += B;
        H_at[static_cast<size_t>(k)] = H;
        aff_at[static_cast<size_t>(k)] = aff;
    }

    const double w_pos = 8.0;
    const double w_vel = 20.0;
    const double w_glide = 30.0;
    Eigen::VectorXd w_state(6);
    w_state << w_pos, w_pos, w_pos, w_vel, w_vel, w_vel;

    const Eigen::MatrixXd& HN = H_at.back();
    const Eigen::VectorXd& aN = aff_at.back();
    Eigen::MatrixXd WH = HN;
    for (int r = 0; r < 6; ++r) WH.row(r) *= w_state(r);
    Eigen::MatrixXd Pu = 2.0 * HN.transpose() * WH;
    Pu = 0.5 * (Pu + Pu.transpose());
    Eigen::VectorXd qu = 2.0 * HN.transpose() * w_state.cwiseProduct(aN);

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_var, n_var);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_var);
    P.topLeftCorner(nU, nU) = Pu;
    q.head(nU) = qu;
    for (int k = 0; k < N; ++k) q(isig(k)) = kScale * dt / kVe;
    for (int s = 0; s < nG; ++s) P(ig(s), ig(s)) = 2.0 * w_glide;
    for (int i = 0; i < n_var; ++i) P(i, i) += 1e-6;

    const int n_point = limit_tilt ? N * 4 : 0;
    const int n_ball = N * n_dir;
    // Final knot: slow, near the pad, and on the ground. These are hard
    // limits so the fuel term cannot buy a landing that is still too fast.
    const int n_term = 6;
    const int nA = n_point + n_ball + nU + nSig + N + 1 + n_term + nG + nG;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nA, n_var);
    Eigen::VectorXd l = Eigen::VectorXd::Constant(nA, -OSQP_INFTY);
    Eigen::VectorXd u = Eigen::VectorXd::Constant(nA, OSQP_INFTY);
    int row = 0;

    if (limit_tilt) {
        for (int k = 0; k < N; ++k) {
            A(row, iu(k, 0)) = 1.0;
            A(row, iu(k, 2)) = -tan_point;
            u(row) = 0.0;
            ++row;
            A(row, iu(k, 0)) = -1.0;
            A(row, iu(k, 2)) = -tan_point;
            u(row) = 0.0;
            ++row;
            A(row, iu(k, 1)) = 1.0;
            A(row, iu(k, 2)) = -tan_point;
            u(row) = 0.0;
            ++row;
            A(row, iu(k, 1)) = -1.0;
            A(row, iu(k, 2)) = -tan_point;
            u(row) = 0.0;
            ++row;
        }
    }

    for (int k = 0; k < N; ++k) {
        for (int d = 0; d < n_dir; ++d) {
            A(row, iu(k, 0)) = dirs[static_cast<size_t>(d)].x();
            A(row, iu(k, 1)) = dirs[static_cast<size_t>(d)].y();
            A(row, iu(k, 2)) = dirs[static_cast<size_t>(d)].z();
            A(row, isig(k)) = -kInnerBall;
            u(row) = 0.0;
            ++row;
        }
    }

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < 3; ++i) {
            const double center = u_ref[static_cast<size_t>(k)](i);
            A(row, iu(k, i)) = 1.0;
            l(row) = (center - rho) / kScale;
            u(row) = (center + rho) / kScale;
            ++row;
        }
    }
    for (int k = 0; k < N; ++k) {
        A(row, isig(k)) = 1.0;
        l(row) = thrust_min / kScale;
        u(row) = thrust_max / kScale;
        ++row;
    }
    for (int k = 0; k < N; ++k) {
        A.block(row, 0, 1, nU) = H_at[static_cast<size_t>(k)].row(2);
        const double zmin = 0.0;
        l(row) = zmin - aff_at[static_cast<size_t>(k)](2);
        u(row) = OSQP_INFTY;
        ++row;
    }
    {
        const Eigen::MatrixXd& HT = H_at.back();
        const Eigen::VectorXd& aT = aff_at.back();
        const double v_cap = 1.0;
        const double xy_cap = 5.0;
        for (int i = 0; i < 3; ++i) {
            A.block(row, 0, 1, nU) = HT.row(3 + i);
            l(row) = -v_cap - aT(3 + i);
            u(row) = v_cap - aT(3 + i);
            ++row;
        }
        for (int i = 0; i < 2; ++i) {
            A.block(row, 0, 1, nU) = HT.row(i);
            l(row) = -xy_cap - aT(i);
            u(row) = xy_cap - aT(i);
            ++row;
        }
        A.block(row, 0, 1, nU) = HT.row(2);
        l(row) = -OSQP_INFTY;
        u(row) = 1.0 - aT(2);
        ++row;
    }
    for (int k = 0; k < N; ++k) A(row, isig(k)) = kScale * dt / kVe;
    l(row) = 0.0;
    u(row) = fuel_avail;
    ++row;

    if (limit_glide) {
        const double pad_radius = 5.0;
        int s = 0;
        const double sign_x[4] = {1.0, -1.0, 0.0, 0.0};
        const double sign_y[4] = {0.0, 0.0, 1.0, -1.0};
        for (int k = 0; k < N; ++k) {
            const Eigen::MatrixXd& Hk = H_at[static_cast<size_t>(k)];
            const Eigen::VectorXd& ak = aff_at[static_cast<size_t>(k)];
            for (int c = 0; c < 4; ++c) {
                A.block(row, 0, 1, nU) =
                    sign_x[c] * Hk.row(0) + sign_y[c] * Hk.row(1) - tan_glide * Hk.row(2);
                A(row, ig(s)) = -1.0;
                u(row) = pad_radius - (
                    sign_x[c] * ak(0) + sign_y[c] * ak(1) - tan_glide * ak(2)
                );
                ++row;
                ++s;
            }
        }
        for (int s_i = 0; s_i < nG; ++s_i) {
            A(row, ig(s_i)) = 1.0;
            l(row) = 0.0;
            ++row;
        }
    }

    if (row != nA) {
        std::cerr << "scvx row count " << row << " expected " << nA << "\n";
        return step;
    }

    csc* P_csc = dense_to_csc_upper(P);
    csc* A_csc = dense_to_csc(A);
    if (P_csc == nullptr || A_csc == nullptr) {
        if (P_csc) c_free(P_csc);
        if (A_csc) c_free(A_csc);
        return step;
    }

    std::vector<c_float> q_vec(static_cast<size_t>(n_var));
    std::vector<c_float> l_vec(static_cast<size_t>(nA));
    std::vector<c_float> u_vec(static_cast<size_t>(nA));
    for (int i = 0; i < n_var; ++i) q_vec[static_cast<size_t>(i)] = q(i);
    for (int i = 0; i < nA; ++i) {
        l_vec[static_cast<size_t>(i)] = l(i);
        u_vec[static_cast<size_t>(i)] = u(i);
    }

    OSQPData data;
    data.n = n_var;
    data.m = nA;
    data.P = P_csc;
    data.q = q_vec.data();
    data.A = A_csc;
    data.l = l_vec.data();
    data.u = u_vec.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_start = 0;
    settings.max_iter = 8000;
    settings.eps_abs = 1e-4;
    settings.eps_rel = 1e-4;
    settings.polish = 0;

    OSQPWorkspace* work = nullptr;
    const c_int setup_status = osqp_setup(&work, &data, &settings);
    if (setup_status != 0 || work == nullptr) {
        c_free(P_csc);
        c_free(A_csc);
        return step;
    }
    osqp_solve(work);
    const bool solved = work->info != nullptr && (
        work->info->status_val == OSQP_SOLVED ||
        work->info->status_val == OSQP_SOLVED_INACCURATE
    ) && work->info->pri_res < 1.0;
    if (!solved) {
        std::cerr << "qp fail N=" << N
                  << " status=" << (work->info ? work->info->status_val : -999)
                  << " pri=" << (work->info ? work->info->pri_res : -1)
                  << "\n";
    }
    if (solved && work->solution != nullptr && work->solution->x != nullptr) {
        step.ok = true;
        step.force.resize(static_cast<size_t>(N));
        for (int k = 0; k < N; ++k) {
            step.force[static_cast<size_t>(k)] = kScale * Eigen::Vector3d(
                work->solution->x[iu(k, 0)],
                work->solution->x[iu(k, 1)],
                work->solution->x[iu(k, 2)]
            );
        }
    }
    osqp_cleanup(work);
    c_free(P_csc);
    c_free(A_csc);
    return step;
}

// The convex plan sometimes stops just above the pad, slow and on target.
// Finish that last bit with a gentle sink so the burn actually reaches the ground.
std::vector<Eigen::Vector3d> append_sink(
    const std::vector<Eigen::Vector3d>& force,
    const Rollout& flew,
    double mass0,
    double pad_x,
    double pad_y,
    double wind_x,
    double wind_y,
    double drag_scale,
    double thrust_min,
    double thrust_max,
    double tan_point
) {
    std::vector<Eigen::Vector3d> out = force;
    if (flew.pos.size() < 2 || flew.vel.empty()) return out;

    Eigen::Vector3d pos = flew.pos.back();
    Eigen::Vector3d vel = flew.vel.back();
    double mass = std::max(MASS_DRY, mass0 - flew.fuel_kg);
    const double miss = std::hypot(pos.x() - pad_x, pos.y() - pad_y);
    if (pos.z() <= 0.5 || pos.z() > 40.0 || miss > 15.0 || vel.norm() > 6.0) {
        return out;
    }

    Eigen::Vector3d last = Eigen::Vector3d(0.0, 0.0, std::clamp(mass * GRAVITY, thrust_min, thrust_max));
    for (int step = 0; step < 30; ++step) {
        const double sink = (pos.z() > 4.0) ? -1.0 : -0.5;
        double az = (sink - vel.z()) / 1.2;
        az = std::clamp(az, -1.2, 1.0);
        const double fz = std::clamp(mass * (GRAVITY + az), thrust_min, thrust_max);
        const double fx = mass * std::clamp(-(pos.x() - pad_x) / 6.0, -1.2, 1.2);
        const double fy = mass * std::clamp(-(pos.y() - pad_y) / 6.0, -1.2, 1.2);
        Eigen::Vector3d u = clip_force(
            Eigen::Vector3d(fx, fy, fz), thrust_min, thrust_max, tan_point
        );
        if (mass <= MASS_DRY + 1e-3) u.setZero();

        const Eigen::Vector3d a = u / std::max(mass, 1.0)
            + Eigen::Vector3d(0.0, 0.0, -GRAVITY)
            + drag_accel(vel, wind_x, wind_y, mass, drag_scale);
        pos += kDt * vel + 0.5 * kDt * kDt * a;
        vel += kDt * a;
        mass = std::max(MASS_DRY, mass - u.norm() * kDt / kVe);
        out.push_back(u);
        last = u;

        const double miss_now = std::hypot(pos.x() - pad_x, pos.y() - pad_y);
        if (pos.z() <= 0.4 && vel.z() > -1.4 && vel.z() < 0.3 && miss_now < 8.0) {
            break;
        }
    }
    // A few extra steps of that same thrust, in case the real rocket is a
    // little behind the point-mass plan.
    for (int extra = 0; extra < 6; ++extra) out.push_back(last);
    return out;
}

struct Attempt {
    bool ok = false;
    Rollout flew;
    std::vector<Eigen::Vector3d> force;
    double tf = 0.0;
};

Attempt plan_duration(
    double tf,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    double mass,
    double pad_x,
    double pad_y,
    double wind_x,
    double wind_y,
    double drag_scale,
    double thrust_min,
    double thrust_max,
    double tan_point,
    double tan_glide
) {
    Attempt attempt;
    const int N = std::max(4, static_cast<int>(std::lround(tf / kDt)));
    attempt.tf = N * kDt;

    // First guess aims at the pad. Hover was leaving the later passes one
    // mass update behind, so the burn they returned never reached the ground.
    const double T = attempt.tf;
    const double t1 = 0.55 * T;
    const double t2 = std::max(1.0e-3, T - t1);
    const double v_touch = -0.5;
    const double z_const = pos.z() + vel.z() * T + 0.5 * t2 * (v_touch - vel.z());
    const double az1 = -2.0 * z_const / std::max(t1 * T, 1.0e-3);
    const double az2 = (v_touch - vel.z() - az1 * t1) / t2;
    // Horizontal accel is a cosine: toward the pad, then back, with no sudden
    // reversal. A hard flip at mid-burn is a landing in the point mass and a
    // flyaway on the real rocket, which cannot yaw the thrust over that fast.
    const double dx = (pad_x - pos.x()) - vel.x() * T;
    const double dy = (pad_y - pos.y()) - vel.y() * T;
    const double ax_amp = dx * (M_PI * M_PI) / std::max(2.0 * T * T, 1.0e-3);
    const double ay_amp = dy * (M_PI * M_PI) / std::max(2.0 * T * T, 1.0e-3);

    // Thrust = mass * the acceleration that reaches the pad. Mass falls during
    // the burn, so a guess built at the starting mass floats the rocket back
    // up once it gets lighter, and the tank is then too empty to finish.
    std::vector<Eigen::Vector3d> u(static_cast<size_t>(N));
    std::vector<double> mass_k(static_cast<size_t>(N), mass);
    for (int iter = 0; iter < 4; ++iter) {
        for (int k = 0; k < N; ++k) {
            const double tk = (static_cast<double>(k) + 0.5) * kDt;
            const double az = (tk < t1) ? az1 : az2;
            const double c = std::cos(M_PI * tk / T);
            const double m_k = mass_k[static_cast<size_t>(k)];
            const Eigen::Vector3d asked = m_k * Eigen::Vector3d(ax_amp * c, ay_amp * c, GRAVITY + az);
            u[static_cast<size_t>(k)] = clip_force(asked, thrust_min, thrust_max, tan_point);
        }
        mass_k = mass_along(u, mass, kDt);
    }

    Attempt best_pass;
    double best_q = 1.0e99;
    Rollout seeded = rollout_forces(
        u, pos, vel, mass, pad_x, pad_y, wind_x, wind_y,
        drag_scale, thrust_min, thrust_max, tan_point, kDt
    );
    best_pass.ok = true;
    best_pass.force = u;
    best_pass.flew = seeded;
    best_pass.tf = attempt.tf;
    best_q = rollout_quality(seeded, attempt.tf);

    double rho = 10000.0;

    for (int pass = 0; pass < kPasses; ++pass) {
        Rollout ref = rollout_forces(
            u, pos, vel, mass, pad_x, pad_y, wind_x, wind_y,
            drag_scale, thrust_min, thrust_max, tan_point, kDt
        );
        std::vector<Eigen::Vector3d> v_ref(static_cast<size_t>(N), vel);
        for (int k = 0; k < N; ++k) {
            const size_t src = std::min(static_cast<size_t>(k), ref.vel.size() - 1);
            v_ref[static_cast<size_t>(k)] = ref.vel[src];
        }

        QpStep qp = solve_qp(
            u, v_ref, pos, vel, mass, pad_x, pad_y, wind_x, wind_y,
            drag_scale, thrust_min, thrust_max, tan_point, tan_glide, rho, kDt
        );
        if (!qp.ok) {
            rho = std::max(500.0, rho * 0.5);
            continue;
        }

        Rollout flew = rollout_forces(
            qp.force, pos, vel, mass, pad_x, pad_y, wind_x, wind_y,
            drag_scale, thrust_min, thrust_max, tan_point, kDt
        );
        // The next pass freezes mass on this thrust. The burn we fly is the
        // pass whose rollout actually lands, which is not always the last one.
        u = qp.force;
        const double q = rollout_quality(flew, attempt.tf);
        if (q < best_q) {
            best_pass.ok = true;
            best_pass.force = u;
            best_pass.flew = flew;
            best_pass.tf = attempt.tf;
            best_q = q;
        }
        if (flew.soft) rho = std::min(thrust_max, rho * 1.5);
        else rho = std::max(1500.0, rho * 0.7);
    }
    return best_pass;
}

}  // namespace

void ScvxGuidance::configure(
    double thrust_min,
    double thrust_max,
    double pointing_cone_deg,
    double gimbal_max_deg,
    double glide_cone_deg,
    double drag_scale
) {
    thrust_min_ = std::max(0.0, thrust_min);
    thrust_max_ = std::max(thrust_min_, thrust_max);
    const double tilt = std::min(pointing_cone_deg, gimbal_max_deg);
    pointing_cone_deg_ = std::clamp(tilt, 0.0, 90.0);
    glide_cone_deg_ = std::clamp(glide_cone_deg, 0.0, 90.0);
    drag_scale_ = std::max(0.0, drag_scale);
    sink_fz_ = 0.0;
    force_.clear();
    plan_ = MpcSolveResult();
}

MpcSolveResult ScvxGuidance::solve(
    double t,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    double mass,
    double pad_x,
    double pad_y,
    double wind_x,
    double wind_y
) {
    const double point_rad = pointing_cone_deg_ * (M_PI / 180.0);
    const double glide_rad = glide_cone_deg_ * (M_PI / 180.0);
    const double tan_point = (pointing_cone_deg_ >= 89.9) ? 1.0e6 : std::tan(point_rad);
    const double tan_glide = (glide_cone_deg_ >= 89.9) ? 1.0e6 : std::tan(glide_rad);

    // Short burns are only useful once the rocket is already low. From the
    // top of the hop they cannot reach the pad.
    const double high_durations[] = {12.0, 15.0, 18.0, 22.0, 26.0};
    const double low_durations[] = {4.0, 6.0, 8.0, 10.0, 12.0};
    const bool low = pos.z() < 80.0;
    const double* durations = low ? low_durations : high_durations;
    const int n_dur = 5;
    Attempt best;
    for (int i_dur = 0; i_dur < n_dur; ++i_dur) {
        const double tf = durations[i_dur];
        Attempt attempt = plan_duration(
            tf, pos, vel, mass, pad_x, pad_y, wind_x, wind_y,
            drag_scale_, thrust_min_, thrust_max_, tan_point, tan_glide
        );
        if (!attempt.ok) continue;
        if (!best.ok || rollout_quality(attempt.flew, attempt.tf) < rollout_quality(best.flew, best.tf)) {
            best = attempt;
        }
    }

    if (best.ok && !best.flew.soft) {
        const std::vector<Eigen::Vector3d> extended = append_sink(
            best.force, best.flew, mass, pad_x, pad_y, wind_x, wind_y,
            drag_scale_, thrust_min_, thrust_max_, tan_point
        );
        if (extended.size() != best.force.size()) {
            best.force = extended;
            best.flew = rollout_forces(
                extended, pos, vel, mass, pad_x, pad_y, wind_x, wind_y,
                drag_scale_, thrust_min_, thrust_max_, tan_point, kDt
            );
        }
    }

    const double touch_t = (best.ok) ? (static_cast<double>(best.flew.pos.size()) - 1.0) * kDt : 0.0;
    const bool on_time = best.ok && touch_t >= best.tf - 2.0;
    // An early graze is not a landing we can fly. The thrust after it is a
    // different burn, and the real rocket follows that burn off the pad.
    const bool landed = best.ok && best.flew.soft && on_time;
    // The point-mass check can touch down in the middle of a long burn.
    // The thrust after that touchdown is a different phase. Flying it on the
    // real rocket, which is a little late, sends the hop the wrong way.
    if (landed) {
        const int flown = static_cast<int>(best.flew.pos.size()) - 1;
        if (flown > 0 && flown < static_cast<int>(best.force.size())) {
            best.force.resize(static_cast<size_t>(flown));
        }
        if (!best.force.empty()) {
            // Padding is a vertical sink, not more of the last divert push.
            // Holding that push past the pad is what flew the rocket away.
            const double end_mass = std::max(MASS_DRY, mass - best.flew.fuel_kg);
            sink_fz_ = std::max(thrust_min_, end_mass * GRAVITY * 0.93);
            const Eigen::Vector3d sink(0.0, 0.0, sink_fz_);
            for (int extra = 0; extra < 6; ++extra) best.force.push_back(sink);
        }
        // The real rocket is still about 2 m/s fast when this short plan
        // says it has stopped. The landings pile up just under the 2 m/s
        // gate and the misses sit just over it. A small extra upward push,
        // the same speed change on every short plan, moves that pile clear.
        if (pos.z() < 80.0 && !best.force.empty()) {
            const double extra_dv = 0.8;
            const double burn_s = static_cast<double>(best.force.size()) * kDt;
            const double extra_fz = mass * extra_dv / std::max(burn_s, 0.5);
            for (Eigen::Vector3d& thrust : best.force) {
                thrust.z() += extra_fz;
                thrust = clip_force(thrust, thrust_min_, thrust_max_, tan_point);
            }
        }
    }
    // A new plan that does not land must not wipe a plan we are already flying.
    // That wipe is what turned a good burn into a straight fall.
    if (landed) {
        plan_ = MpcSolveResult();
        plan_.ok = true;
        plan_.plan_pos = best.flew.pos;
        t0_ = t;
        dt_ = kDt;
        force_ = best.force;
        plan_.force = force_.front();
    } else if (force_.empty()) {
        plan_ = MpcSolveResult();
        plan_.ok = false;
        t0_ = t;
        dt_ = kDt;
        const double hover = std::clamp(mass * GRAVITY, thrust_min_, thrust_max_);
        force_.push_back(Eigen::Vector3d(0.0, 0.0, hover));
        plan_.force = force_.front();
        plan_.plan_pos.push_back(pos);
    }

    const double end_z = (best.ok && !best.flew.pos.empty()) ? best.flew.pos.back().z() : -1.0;
    std::cerr << "scvx t=" << t
              << " tf=" << (best.ok ? best.tf : 0.0)
              << " fuel_kg=" << (best.ok ? best.flew.fuel_kg : -1.0)
              << " miss=" << (best.ok ? best.flew.miss : -1.0)
              << " vz=" << (best.ok ? best.flew.vz : 0.0)
              << " z=" << end_z
              << " soft=" << (landed ? 1 : 0)
              << "\n";
    return plan_;
}

Eigen::Vector3d ScvxGuidance::force_at(double t_query) const {
    if (force_.empty()) {
        return Eigen::Vector3d(0.0, 0.0, thrust_min_);
    }
    const double elapsed = t_query - t0_;
    if (elapsed <= 0.0) return force_.front();
    const int k = static_cast<int>(elapsed / dt_);
    if (k >= static_cast<int>(force_.size())) {
        if (!plan_.ok || sink_fz_ <= 0.0) return force_.back();
        return Eigen::Vector3d(0.0, 0.0, sink_fz_);
    }
    return force_[static_cast<size_t>(k)];
}

}  // namespace icarus
