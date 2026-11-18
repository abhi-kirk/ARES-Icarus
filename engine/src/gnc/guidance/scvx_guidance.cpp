#include "gnc/guidance/scvx_guidance.h"

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <qoco.h>

namespace icarus {

namespace {

const double kVe = MAX_THRUST / MAX_FLOW_RATE;
const double kDt = 0.5;
const int kPasses = 5;

// One straight cut, a'x <= h. Columns are the decision variables that cut touches.
struct ConeCut {
    std::vector<int> cols;
    std::vector<double> vals;
    double h = 0.0;
};

void add_coeff(ConeCut& cut, int col, double value) {
    if (std::abs(value) < 1.0e-16) return;
    cut.cols.push_back(col);
    cut.vals.push_back(value);
}

// sign * (one row of H), which is a cut on the thrust list.
ConeCut cut_from_row(const Eigen::MatrixXd& H, int row, double sign) {
    ConeCut cut;
    for (int col = 0; col < H.cols(); ++col) {
        add_coeff(cut, col, sign * H(row, col));
    }
    return cut;
}

struct OwnedCsc {
    std::vector<QOCOFloat> x;
    std::vector<QOCOInt> i;
    std::vector<QOCOInt> p;
    QOCOCscMatrix mat{};
};

// Upper triangle, including the diagonal. QOCO reads P that way.
OwnedCsc upper_triangle(const Eigen::MatrixXd& dense) {
    OwnedCsc out;
    const int n = static_cast<int>(dense.rows());
    out.p.assign(static_cast<size_t>(n + 1), 0);
    for (int col = 0; col < n; ++col) {
        out.p[static_cast<size_t>(col)] = static_cast<QOCOInt>(out.x.size());
        for (int row = 0; row <= col; ++row) {
            const double value = dense(row, col);
            if (std::abs(value) < 1.0e-16) continue;
            out.i.push_back(row);
            out.x.push_back(value);
        }
    }
    out.p[static_cast<size_t>(n)] = static_cast<QOCOInt>(out.x.size());
    qoco_set_csc(
        &out.mat, n, n, static_cast<QOCOInt>(out.x.size()),
        out.x.data(), out.p.data(), out.i.data()
    );
    return out;
}

OwnedCsc cuts_to_csc(const std::vector<ConeCut>& cuts, int n_var) {
    OwnedCsc out;
    const int n_rows = static_cast<int>(cuts.size());
    std::vector<int> count(static_cast<size_t>(n_var), 0);
    for (const ConeCut& cut : cuts) {
        for (int col : cut.cols) count[static_cast<size_t>(col)] += 1;
    }
    out.p.assign(static_cast<size_t>(n_var + 1), 0);
    for (int col = 0; col < n_var; ++col) {
        out.p[static_cast<size_t>(col + 1)] =
            out.p[static_cast<size_t>(col)] + count[static_cast<size_t>(col)];
    }
    const int nnz = static_cast<int>(out.p.back());
    out.x.assign(static_cast<size_t>(nnz), 0.0);
    out.i.assign(static_cast<size_t>(nnz), 0);
    std::vector<int> next(out.p.begin(), out.p.end() - 1);
    for (int row = 0; row < n_rows; ++row) {
        const ConeCut& cut = cuts[static_cast<size_t>(row)];
        for (size_t t = 0; t < cut.cols.size(); ++t) {
            const int dest = next[static_cast<size_t>(cut.cols[t])];
            next[static_cast<size_t>(cut.cols[t])] = dest + 1;
            out.i[static_cast<size_t>(dest)] = row;
            out.x[static_cast<size_t>(dest)] = cut.vals[t];
        }
    }
    qoco_set_csc(
        &out.mat, n_rows, n_var, nnz,
        out.x.data(), out.p.data(), out.i.data()
    );
    return out;
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
    // The tracking cost is huge next to the thrust bounds. Scaling the whole
    // cost leaves the best thrust unchanged and keeps the cone solver's
    // Newton steps in range.
    const double cost_scale = 1.0 / std::max(1.0, std::max(P.norm(), q.norm()));
    P *= cost_scale;
    q *= cost_scale;

    // Straight cuts first. The ball comes after them, one cone per knot.
    // QOCO wants h - Gx inside the cone, so a'x <= rhs is stored as that row of G.
    std::vector<ConeCut> cuts;

    if (limit_tilt) {
        for (int k = 0; k < N; ++k) {
            const int axis[4] = {0, 0, 1, 1};
            const double sign[4] = {1.0, -1.0, 1.0, -1.0};
            for (int c = 0; c < 4; ++c) {
                ConeCut cut;
                add_coeff(cut, iu(k, axis[c]), sign[c]);
                add_coeff(cut, iu(k, 2), -tan_point);
                cut.h = 0.0;
                cuts.push_back(cut);
            }
        }
    }

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < 3; ++i) {
            const double center = u_ref[static_cast<size_t>(k)](i);
            const double lo = (center - rho) / kScale;
            const double hi = (center + rho) / kScale;
            ConeCut upper;
            add_coeff(upper, iu(k, i), 1.0);
            upper.h = hi;
            cuts.push_back(upper);
            ConeCut lower;
            add_coeff(lower, iu(k, i), -1.0);
            lower.h = -lo;
            cuts.push_back(lower);
        }
    }
    for (int k = 0; k < N; ++k) {
        const double lo = thrust_min / kScale;
        const double hi = thrust_max / kScale;
        ConeCut upper;
        add_coeff(upper, isig(k), 1.0);
        upper.h = hi;
        cuts.push_back(upper);
        ConeCut lower;
        add_coeff(lower, isig(k), -1.0);
        lower.h = -lo;
        cuts.push_back(lower);
    }
    for (int k = 0; k < N; ++k) {
        // Altitude stays above the ground: H z + aff >= 0.
        ConeCut above = cut_from_row(H_at[static_cast<size_t>(k)], 2, -1.0);
        above.h = aff_at[static_cast<size_t>(k)](2);
        cuts.push_back(above);
    }
    {
        // Final knot: slow, near the pad, and on the ground. These are hard
        // limits so the fuel term cannot buy a landing that is still too fast.
        const Eigen::MatrixXd& HT = H_at.back();
        const Eigen::VectorXd& aT = aff_at.back();
        const double v_cap = 1.0;
        const double xy_cap = 5.0;
        for (int i = 0; i < 3; ++i) {
            ConeCut upper = cut_from_row(HT, 3 + i, 1.0);
            upper.h = v_cap - aT(3 + i);
            cuts.push_back(upper);
            ConeCut lower = cut_from_row(HT, 3 + i, -1.0);
            lower.h = v_cap + aT(3 + i);
            cuts.push_back(lower);
        }
        for (int i = 0; i < 2; ++i) {
            ConeCut upper = cut_from_row(HT, i, 1.0);
            upper.h = xy_cap - aT(i);
            cuts.push_back(upper);
            ConeCut lower = cut_from_row(HT, i, -1.0);
            lower.h = xy_cap + aT(i);
            cuts.push_back(lower);
        }
        ConeCut on_ground = cut_from_row(HT, 2, 1.0);
        on_ground.h = 1.0 - aT(2);
        cuts.push_back(on_ground);
    }
    {
        ConeCut under;
        ConeCut over;
        for (int k = 0; k < N; ++k) {
            const double per_step = kScale * dt / kVe;
            add_coeff(under, isig(k), -per_step);
            add_coeff(over, isig(k), per_step);
        }
        under.h = 0.0;
        over.h = fuel_avail;
        cuts.push_back(under);
        cuts.push_back(over);
    }

    if (limit_glide) {
        const double pad_radius = 5.0;
        int s = 0;
        const double sign_x[4] = {1.0, -1.0, 0.0, 0.0};
        const double sign_y[4] = {0.0, 0.0, 1.0, -1.0};
        for (int k = 0; k < N; ++k) {
            const Eigen::MatrixXd& Hk = H_at[static_cast<size_t>(k)];
            const Eigen::VectorXd& ak = aff_at[static_cast<size_t>(k)];
            for (int c = 0; c < 4; ++c) {
                ConeCut glide;
                for (int col = 0; col < nU; ++col) {
                    const double coeff = sign_x[c] * Hk(0, col)
                        + sign_y[c] * Hk(1, col)
                        - tan_glide * Hk(2, col);
                    add_coeff(glide, col, coeff);
                }
                add_coeff(glide, ig(s), -1.0);
                glide.h = pad_radius - (
                    sign_x[c] * ak(0) + sign_y[c] * ak(1) - tan_glide * ak(2)
                );
                cuts.push_back(glide);
                ++s;
            }
        }
        for (int s_i = 0; s_i < nG; ++s_i) {
            ConeCut slack;
            add_coeff(slack, ig(s_i), -1.0);
            slack.h = 0.0;
            cuts.push_back(slack);
        }
    }

    const int n_linear = static_cast<int>(cuts.size());
    // One ball per knot: slack >= length of the thrust arrow. Same scale on
    // both, so the 1000 that keeps the numbers small cancels out.
    for (int k = 0; k < N; ++k) {
        ConeCut sig;
        add_coeff(sig, isig(k), -1.0);
        sig.h = 0.0;
        cuts.push_back(sig);
        for (int i = 0; i < 3; ++i) {
            ConeCut axis;
            add_coeff(axis, iu(k, i), -1.0);
            axis.h = 0.0;
            cuts.push_back(axis);
        }
    }

    OwnedCsc P_csc = upper_triangle(P);
    OwnedCsc G_csc = cuts_to_csc(cuts, n_var);
    std::vector<QOCOFloat> q_vec(static_cast<size_t>(n_var));
    std::vector<QOCOFloat> h_vec(cuts.size());
    for (int i = 0; i < n_var; ++i) q_vec[static_cast<size_t>(i)] = q(i);
    for (size_t r = 0; r < cuts.size(); ++r) h_vec[r] = cuts[r].h;

    std::vector<QOCOInt> cone_dim(static_cast<size_t>(N), 4);
    QOCOSettings settings;
    set_default_settings(&settings);
    settings.verbose = 0;
    settings.max_iters = 200;
    settings.ruiz_iters = 5;

    QOCOSolver* solver = static_cast<QOCOSolver*>(std::calloc(1, sizeof(QOCOSolver)));
    const QOCOInt setup_status = qoco_setup(
        solver,
        n_var,
        static_cast<QOCOInt>(cuts.size()),
        0,
        &P_csc.mat,
        q_vec.data(),
        nullptr,
        nullptr,
        &G_csc.mat,
        h_vec.data(),
        n_linear,
        N,
        cone_dim.data(),
        &settings
    );
    if (setup_status != QOCO_NO_ERROR) {
        std::cerr << "cone setup fail N=" << N << " status=" << setup_status << "\n";
        std::free(solver);
        return step;
    }
    const QOCOInt solve_status = qoco_solve(solver);
    const bool solved = solver->sol != nullptr && (
        solve_status == QOCO_SOLVED || solve_status == QOCO_SOLVED_INACCURATE
    );
    bool cone_ok = solved && solver->sol->x != nullptr;
    if (cone_ok) {
        for (int k = 0; k < N; ++k) {
            const double ux = solver->sol->x[iu(k, 0)];
            const double uy = solver->sol->x[iu(k, 1)];
            const double uz = solver->sol->x[iu(k, 2)];
            const double sig = solver->sol->x[isig(k)];
            const double length = std::sqrt(ux * ux + uy * uy + uz * uz);
            if (sig < -1.0e-3 || length > sig + 1.0e-2) cone_ok = false;
        }
    }
    if (!cone_ok) {
        std::cerr << "cone fail N=" << N
                  << " status=" << solve_status
                  << " pres=" << (solver->sol ? solver->sol->pres : -1)
                  << "\n";
    }
    if (cone_ok) {
        step.ok = true;
        step.force.resize(static_cast<size_t>(N));
        for (int k = 0; k < N; ++k) {
            step.force[static_cast<size_t>(k)] = kScale * Eigen::Vector3d(
                solver->sol->x[iu(k, 0)],
                solver->sol->x[iu(k, 1)],
                solver->sol->x[iu(k, 2)]
            );
        }
    }
    qoco_cleanup(solver);
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
        // The real rocket is faster than this point mass when the plan is
        // flown open loop. A short plan is about 2 m/s fast. A long plan,
        // with no later correction, is about 3 m/s fast. The same extra
        // upward push on every plan of that kind keeps the touchdown inside
        // the 2 m/s gate.
        if (!best.force.empty()) {
            const double extra_dv = (pos.z() < 80.0) ? 0.8 : 3.0;
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
