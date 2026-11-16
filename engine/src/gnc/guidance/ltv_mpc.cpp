#include "gnc/guidance/ltv_mpc.h"

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <osqp.h>

namespace icarus {

namespace {

// OSQP's csc_matrix only stores pointers. Build an owned CSC so the buffers
// live until osqp_cleanup frees them.
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

// Pack a dense symmetric matrix into OSQP's upper-triangular CSC form.
csc* dense_to_csc_upper(const Eigen::MatrixXd& dense) {
    const int n = static_cast<int>(dense.rows());
    std::vector<c_int> row_idx;
    std::vector<c_float> values;
    std::vector<c_int> col_ptr(n + 1, 0);

    for (int col = 0; col < n; ++col) {
        col_ptr[col] = static_cast<c_int>(values.size());
        for (int row = 0; row <= col; ++row) {
            const double v = dense(row, col);
            if (std::abs(v) < 1e-16) continue;
            row_idx.push_back(row);
            values.push_back(v);
        }
    }
    col_ptr[n] = static_cast<c_int>(values.size());
    return owned_csc_from_triplets(n, n, row_idx, values, col_ptr);
}

csc* dense_to_csc(const Eigen::MatrixXd& dense) {
    const int rows = static_cast<int>(dense.rows());
    const int cols = static_cast<int>(dense.cols());
    std::vector<c_int> row_idx;
    std::vector<c_float> values;
    std::vector<c_int> col_ptr(cols + 1, 0);

    for (int col = 0; col < cols; ++col) {
        col_ptr[col] = static_cast<c_int>(values.size());
        for (int row = 0; row < rows; ++row) {
            const double v = dense(row, col);
            if (std::abs(v) < 1e-16) continue;
            row_idx.push_back(row);
            values.push_back(v);
        }
    }
    col_ptr[cols] = static_cast<c_int>(values.size());
    return owned_csc_from_triplets(rows, cols, row_idx, values, col_ptr);
}

}  // namespace

void LtvMpc::configure(
    int horizon,
    double dt,
    double w_pos,
    double w_vel,
    double w_u,
    double w_terminal,
    double thrust_min,
    double thrust_max,
    double pointing_cone_deg
) {
    horizon_ = std::max(2, horizon);
    dt_ = std::max(0.05, dt);
    w_pos_ = w_pos;
    w_vel_ = w_vel;
    w_u_ = w_u;
    w_terminal_ = w_terminal;
    thrust_min_ = thrust_min;
    thrust_max_ = thrust_max;
    pointing_cone_deg_ = pointing_cone_deg;
}

void LtvMpc::set_reference(const SoftLandReference* reference) {
    reference_ = reference;
}

MpcSolveResult LtvMpc::solve(
    double t,
    const Eigen::Vector3d& state_pos,
    const Eigen::Vector3d& state_vel,
    double mass,
    double pad_x,
    double pad_y
) {
    MpcSolveResult result;
    result.plan_pos.push_back(state_pos);

    const double m = std::max(mass, 1.0);
    const int N = horizon_;
    const int nx = 6;
    const int nu = 3;
    const int nU = N * nu;
    const int nX = N * nx;

    // Discrete double integrator with gravity on z.
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    A.block(0, 3, 3, 3) = dt_ * Eigen::Matrix3d::Identity();

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    B.block(0, 0, 3, 3) = (0.5 * dt_ * dt_ / m) * Eigen::Matrix3d::Identity();
    B.block(3, 0, 3, 3) = (dt_ / m) * Eigen::Matrix3d::Identity();

    Eigen::VectorXd c = Eigen::VectorXd::Zero(nx);
    c(2) = -0.5 * dt_ * dt_ * GRAVITY;
    c(5) = -dt_ * GRAVITY;

    // Current pad-relative state.
    Eigen::VectorXd x0(nx);
    x0 << state_pos.x() - pad_x,
          state_pos.y() - pad_y,
          state_pos.z(),
          state_vel.x(),
          state_vel.y(),
          state_vel.z();

    // Build G, H, d so X = G x0 + H U + d, with X = [x1; ...; xN].
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nX, nx);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nX, nU);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(nX);

    Eigen::MatrixXd A_pow = A;
    Eigen::VectorXd d_roll = c;
    for (int k = 0; k < N; ++k) {
        G.block(k * nx, 0, nx, nx) = A_pow;
        d.segment(k * nx, nx) = d_roll;

        // H block: x_{k+1} depends on u_0 .. u_k
        Eigen::MatrixXd A_to_u = Eigen::MatrixXd::Identity(nx, nx);
        for (int j = k; j >= 0; --j) {
            H.block(k * nx, j * nu, nx, nu) = A_to_u * B;
            A_to_u = A_to_u * A;
        }

        d_roll = A * d_roll + c;
        A_pow = A * A_pow;
    }

    // Reference stack and stage weights.
    Eigen::VectorXd Xref = Eigen::VectorXd::Zero(nX);
    Eigen::VectorXd Uref = Eigen::VectorXd::Zero(nU);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nX, nX);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nU, nU);

    for (int k = 0; k < N; ++k) {
        const double tk = t + (k + 1) * dt_;
        ReferenceKnot knot;
        if (reference_ != nullptr) {
            knot = reference_->at(tk);
        } else {
            knot.pos = Eigen::Vector3d(pad_x, pad_y, 0.0);
            knot.force = Eigen::Vector3d(0.0, 0.0, m * GRAVITY);
        }

        Xref.segment(k * nx, nx) << knot.pos.x() - pad_x,
                                    knot.pos.y() - pad_y,
                                    std::max(0.0, knot.pos.z()),
                                    knot.vel.x(),
                                    knot.vel.y(),
                                    knot.vel.z();
        Uref.segment(k * nu, nu) = knot.force;

        const double w_p = (k == N - 1) ? w_terminal_ * w_pos_ : w_pos_;
        const double w_v = (k == N - 1) ? w_terminal_ * w_vel_ : w_vel_;
        Q.block(k * nx + 0, k * nx + 0, 3, 3) = w_p * Eigen::Matrix3d::Identity();
        Q.block(k * nx + 3, k * nx + 3, 3, 3) = w_v * Eigen::Matrix3d::Identity();
        R.block(k * nu, k * nu, nu, nu) = w_u_ * Eigen::Matrix3d::Identity();
    }

    // Cost (1/2) U^T P U + q^T U  with
    // P = 2 (H^T Q H + R),  q = 2 H^T Q (G x0 + d - Xref) - 2 R Uref.
    const Eigen::VectorXd bias = G * x0 + d - Xref;
    Eigen::MatrixXd Pdense = 2.0 * (H.transpose() * Q * H + R);
    // Keep P symmetric positive definite for OSQP.
    Pdense = 0.5 * (Pdense + Pdense.transpose());
    for (int i = 0; i < nU; ++i) {
        Pdense(i, i) += 1e-8;
    }
    Eigen::VectorXd qdense = 2.0 * H.transpose() * Q * bias - 2.0 * R * Uref;

    // Constraints: box on Fx,Fy,Fz plus a linear pointing cone on each step.
    //   -Tmax <= Fx,Fy <= Tmax
    //    Tmin <= Fz    <= Tmax
    //   |Fx| <= tan(cone) * Fz   (and same for Fy) when cone < 90
    const double cone_rad = pointing_cone_deg_ * (3.14159265358979323846 / 180.0);
    const double tan_cone = (pointing_cone_deg_ >= 89.9) ? 1.0e6 : std::tan(cone_rad);
    // Cone extras: Fx - tan Fz <= 0, -Fx - tan Fz <= 0, and the same for Fy.
    const int n_cone = (pointing_cone_deg_ < 89.9) ? N * 4 : 0;
    const int n_cons = n_cone;

    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(std::max(1, n_cons), nU);
    Eigen::VectorXd lineq = Eigen::VectorXd::Constant(std::max(1, n_cons), -OSQP_INFTY);
    Eigen::VectorXd uineq = Eigen::VectorXd::Constant(std::max(1, n_cons), OSQP_INFTY);

    if (n_cone > 0) {
        for (int k = 0; k < N; ++k) {
            const int c0 = k * 4;
            const int u0 = k * nu;
            // Fx - tan Fz <= 0
            Aineq(c0 + 0, u0 + 0) = 1.0;
            Aineq(c0 + 0, u0 + 2) = -tan_cone;
            uineq(c0 + 0) = 0.0;
            // -Fx - tan Fz <= 0
            Aineq(c0 + 1, u0 + 0) = -1.0;
            Aineq(c0 + 1, u0 + 2) = -tan_cone;
            uineq(c0 + 1) = 0.0;
            // Fy - tan Fz <= 0
            Aineq(c0 + 2, u0 + 1) = 1.0;
            Aineq(c0 + 2, u0 + 2) = -tan_cone;
            uineq(c0 + 2) = 0.0;
            // -Fy - tan Fz <= 0
            Aineq(c0 + 3, u0 + 1) = -1.0;
            Aineq(c0 + 3, u0 + 2) = -tan_cone;
            uineq(c0 + 3) = 0.0;
            lineq(c0 + 0) = -OSQP_INFTY;
            lineq(c0 + 1) = -OSQP_INFTY;
            lineq(c0 + 2) = -OSQP_INFTY;
            lineq(c0 + 3) = -OSQP_INFTY;
        }
    }

    // Variable bounds.
    std::vector<c_float> l_bounds(nU);
    std::vector<c_float> u_bounds(nU);
    for (int k = 0; k < N; ++k) {
        l_bounds[k * nu + 0] = -thrust_max_;
        u_bounds[k * nu + 0] = thrust_max_;
        l_bounds[k * nu + 1] = -thrust_max_;
        u_bounds[k * nu + 1] = thrust_max_;
        l_bounds[k * nu + 2] = thrust_min_;
        u_bounds[k * nu + 2] = thrust_max_;
    }

    // OSQP wants one constraint matrix. Stack identity (for bounds) with cone.
    // Easier: use OSQP's data->l/u on A=I for bounds only when no cone; with cone
    // use [I; Aineq].
    const int nA = nU + n_cons;
    Eigen::MatrixXd Afull = Eigen::MatrixXd::Zero(nA, nU);
    Afull.block(0, 0, nU, nU) = Eigen::MatrixXd::Identity(nU, nU);
    if (n_cons > 0) {
        Afull.block(nU, 0, n_cons, nU) = Aineq.topRows(n_cons);
    }
    Eigen::VectorXd lfull(nA);
    Eigen::VectorXd ufull(nA);
    for (int i = 0; i < nU; ++i) {
        lfull(i) = l_bounds[i];
        ufull(i) = u_bounds[i];
    }
    for (int i = 0; i < n_cons; ++i) {
        lfull(nU + i) = lineq(i);
        ufull(nU + i) = uineq(i);
    }

    csc* P_csc = dense_to_csc_upper(Pdense);
    csc* A_csc = dense_to_csc(Afull);
    if (P_csc == nullptr || A_csc == nullptr) {
        if (P_csc) c_free(P_csc);
        if (A_csc) c_free(A_csc);
        // Fallback: hover thrust.
        result.force = Eigen::Vector3d(0.0, 0.0, m * GRAVITY);
        return result;
    }

    std::vector<c_float> q_vec(nU);
    std::vector<c_float> l_vec(nA);
    std::vector<c_float> u_vec(nA);
    for (int i = 0; i < nU; ++i) q_vec[i] = qdense(i);
    for (int i = 0; i < nA; ++i) {
        l_vec[i] = lfull(i);
        u_vec[i] = ufull(i);
    }

    OSQPData data;
    data.n = nU;
    data.m = nA;
    data.P = P_csc;
    data.q = q_vec.data();
    data.A = A_csc;
    data.l = l_vec.data();
    data.u = u_vec.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_start = 1;
    settings.max_iter = 4000;
    settings.eps_abs = 1e-4;
    settings.eps_rel = 1e-4;

    OSQPWorkspace* work = nullptr;
    c_int setup_status = osqp_setup(&work, &data, &settings);
    if (setup_status != 0 || work == nullptr) {
        c_free(P_csc);
        c_free(A_csc);
        result.force = Eigen::Vector3d(0.0, 0.0, m * GRAVITY);
        return result;
    }

    osqp_solve(work);

    const bool solved =
        work->info->status_val == OSQP_SOLVED ||
        work->info->status_val == OSQP_SOLVED_INACCURATE;

    Eigen::VectorXd U = Uref;
    if (solved && work->solution != nullptr && work->solution->x != nullptr) {
        for (int i = 0; i < nU; ++i) {
            U(i) = work->solution->x[i];
        }
        result.ok = true;
    }

    result.force = U.segment(0, 3);

    // Roll the predicted states for the plan trail (inertial).
    Eigen::VectorXd x = x0;
    for (int k = 0; k < N; ++k) {
        Eigen::Vector3d uk = U.segment(k * nu, 3);
        x = A * x + B * uk + c;
        Eigen::Vector3d pos_i(x(0) + pad_x, x(1) + pad_y, std::max(0.0, x(2)));
        result.plan_pos.push_back(pos_i);
        if (x(2) <= 0.0) break;
    }

    osqp_cleanup(work);
    // osqp_cleanup frees P and A that were given to setup.
    return result;
}

}  // namespace icarus
