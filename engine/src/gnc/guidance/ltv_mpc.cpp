#include "gnc/guidance/ltv_mpc.h"

#include "dynamics/constants.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <osqp.h>

namespace icarus {

namespace {

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
    double pointing_cone_deg,
    double glide_cone_deg
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
    glide_cone_deg_ = glide_cone_deg;
    have_warm_ = false;
    last_U_.resize(0);
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

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    A.block(0, 3, 3, 3) = dt_ * Eigen::Matrix3d::Identity();

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    B.block(0, 0, 3, 3) = (0.5 * dt_ * dt_ / m) * Eigen::Matrix3d::Identity();
    B.block(3, 0, 3, 3) = (dt_ / m) * Eigen::Matrix3d::Identity();

    Eigen::VectorXd c = Eigen::VectorXd::Zero(nx);
    c(2) = -0.5 * dt_ * dt_ * GRAVITY;
    c(5) = -dt_ * GRAVITY;

    Eigen::VectorXd x0(nx);
    x0 << state_pos.x() - pad_x,
          state_pos.y() - pad_y,
          state_pos.z(),
          state_vel.x(),
          state_vel.y(),
          state_vel.z();

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nX, nx);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nX, nU);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(nX);

    Eigen::MatrixXd A_pow = A;
    Eigen::VectorXd d_roll = c;
    for (int k = 0; k < N; ++k) {
        G.block(k * nx, 0, nx, nx) = A_pow;
        d.segment(k * nx, nx) = d_roll;

        Eigen::MatrixXd A_to_u = Eigen::MatrixXd::Identity(nx, nx);
        for (int j = k; j >= 0; --j) {
            H.block(k * nx, j * nu, nx, nu) = A_to_u * B;
            A_to_u = A_to_u * A;
        }

        d_roll = A * d_roll + c;
        A_pow = A * A_pow;
    }

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

    const Eigen::VectorXd affine = G * x0 + d;
    const Eigen::VectorXd bias = affine - Xref;
    Eigen::MatrixXd Pu = 2.0 * (H.transpose() * Q * H + R);
    Pu = 0.5 * (Pu + Pu.transpose());
    for (int i = 0; i < nU; ++i) {
        Pu(i, i) += 1e-8;
    }
    Eigen::VectorXd qu = 2.0 * H.transpose() * Q * bias - 2.0 * R * Uref;

    // Pointing cone on each control: |Fx|,|Fy| <= tan(point) * Fz (hard).
    const double point_rad = pointing_cone_deg_ * (3.14159265358979323846 / 180.0);
    const double tan_point = (pointing_cone_deg_ >= 89.9) ? 1.0e6 : std::tan(point_rad);
    const int n_point = (pointing_cone_deg_ < 89.9) ? N * 4 : 0;

    // Glide corridor on each predicted state:
    //   |px|,|py| <= tan(glide) * z + pad_radius
    // Soft: A_glide U - s <= b, s >= 0, with a large quadratic penalty on s.
    // pad_radius keeps a finite landing zone so the cone does not collapse to a
    // point at z→0 (which otherwise makes the vehicle climb to regain margin).
    const double glide_rad = glide_cone_deg_ * (3.14159265358979323846 / 180.0);
    const double tan_glide = (glide_cone_deg_ >= 89.9) ? 1.0e6 : std::tan(glide_rad);
    const double pad_radius = 5.0;
    const int n_glide = (glide_cone_deg_ < 89.9) ? N * 4 : 0;
    const double w_glide_slack = 2.0e3;

    // Decision vector z = [U; s]. Pointing stays hard on U only.
    const int n_var = nU + n_glide;
    Eigen::MatrixXd Pdense = Eigen::MatrixXd::Zero(n_var, n_var);
    Pdense.topLeftCorner(nU, nU) = Pu;
    for (int i = 0; i < n_glide; ++i) {
        Pdense(nU + i, nU + i) = 2.0 * w_glide_slack;
    }
    Eigen::VectorXd qdense = Eigen::VectorXd::Zero(n_var);
    qdense.head(nU) = qu;

    // Rows: thrust bounds (nU) + pointing (n_point) + soft glide (n_glide) + s >= 0 (n_glide).
    const int nA = nU + n_point + n_glide + n_glide;
    Eigen::MatrixXd Afull = Eigen::MatrixXd::Zero(std::max(1, nA), n_var);
    Eigen::VectorXd lfull = Eigen::VectorXd::Constant(std::max(1, nA), -OSQP_INFTY);
    Eigen::VectorXd ufull = Eigen::VectorXd::Constant(std::max(1, nA), OSQP_INFTY);

    // Thrust box on U.
    Afull.block(0, 0, nU, nU) = Eigen::MatrixXd::Identity(nU, nU);
    for (int k = 0; k < N; ++k) {
        lfull(k * nu + 0) = -thrust_max_;
        ufull(k * nu + 0) = thrust_max_;
        lfull(k * nu + 1) = -thrust_max_;
        ufull(k * nu + 1) = thrust_max_;
        lfull(k * nu + 2) = thrust_min_;
        ufull(k * nu + 2) = thrust_max_;
    }

    int row = nU;
    if (n_point > 0) {
        for (int k = 0; k < N; ++k) {
            const int u0 = k * nu;
            Afull(row, u0 + 0) = 1.0;
            Afull(row, u0 + 2) = -tan_point;
            ufull(row) = 0.0;
            ++row;
            Afull(row, u0 + 0) = -1.0;
            Afull(row, u0 + 2) = -tan_point;
            ufull(row) = 0.0;
            ++row;
            Afull(row, u0 + 1) = 1.0;
            Afull(row, u0 + 2) = -tan_point;
            ufull(row) = 0.0;
            ++row;
            Afull(row, u0 + 1) = -1.0;
            Afull(row, u0 + 2) = -tan_point;
            ufull(row) = 0.0;
            ++row;
        }
    }

    if (n_glide > 0) {
        int s_i = 0;
        for (int k = 0; k < N; ++k) {
            // A_glide U - s <= b  =>  (e_x - tan e_z)^T (affine + H U) - s <= 0
            const int x_row = k * nx + 0;
            const int y_row = k * nx + 1;
            const int z_row = k * nx + 2;
            Eigen::RowVectorXd Hx = H.row(x_row);
            Eigen::RowVectorXd Hy = H.row(y_row);
            Eigen::RowVectorXd Hz = H.row(z_row);
            const double ax = affine(x_row);
            const double ay = affine(y_row);
            const double az = affine(z_row);

            // px - tan*z <= pad_radius  =>  Hx U - tan Hz U - s <= -ax + tan az + pad
            Afull.block(row, 0, 1, nU) = Hx - tan_glide * Hz;
            Afull(row, nU + s_i) = -1.0;
            ufull(row) = -ax + tan_glide * az + pad_radius;
            ++row;
            ++s_i;

            Afull.block(row, 0, 1, nU) = -Hx - tan_glide * Hz;
            Afull(row, nU + s_i) = -1.0;
            ufull(row) = ax + tan_glide * az + pad_radius;
            ++row;
            ++s_i;

            Afull.block(row, 0, 1, nU) = Hy - tan_glide * Hz;
            Afull(row, nU + s_i) = -1.0;
            ufull(row) = -ay + tan_glide * az + pad_radius;
            ++row;
            ++s_i;

            Afull.block(row, 0, 1, nU) = -Hy - tan_glide * Hz;
            Afull(row, nU + s_i) = -1.0;
            ufull(row) = ay + tan_glide * az + pad_radius;
            ++row;
            ++s_i;
        }

        // s >= 0
        for (int i = 0; i < n_glide; ++i) {
            Afull(row, nU + i) = 1.0;
            lfull(row) = 0.0;
            ufull(row) = OSQP_INFTY;
            ++row;
        }
    }

    csc* P_csc = dense_to_csc_upper(Pdense);
    csc* A_csc = dense_to_csc(Afull.topRows(nA));
    if (P_csc == nullptr || A_csc == nullptr) {
        if (P_csc) c_free(P_csc);
        if (A_csc) c_free(A_csc);
        // Fall back to the reference ask, not pure hover.
        result.force = Uref.segment(0, 3);
        return result;
    }

    std::vector<c_float> q_vec(n_var);
    std::vector<c_float> l_vec(nA);
    std::vector<c_float> u_vec(nA);
    for (int i = 0; i < n_var; ++i) q_vec[i] = qdense(i);
    for (int i = 0; i < nA; ++i) {
        l_vec[i] = lfull(i);
        u_vec[i] = ufull(i);
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
    settings.warm_start = 1;
    settings.max_iter = 4000;
    settings.eps_abs = 1e-4;
    settings.eps_rel = 1e-4;

    OSQPWorkspace* work = nullptr;
    c_int setup_status = osqp_setup(&work, &data, &settings);
    if (setup_status != 0 || work == nullptr) {
        c_free(P_csc);
        c_free(A_csc);
        result.force = Uref.segment(0, 3);
        return result;
    }

    // Shift the previous U left by one step and warm-start OSQP (U only; slacks 0).
    std::vector<c_float> warm(n_var, 0.0);
    if (have_warm_ && last_U_.size() == nU) {
        for (int i = 0; i < nU - nu; ++i) {
            warm[i] = last_U_(i + nu);
        }
        for (int i = 0; i < nu; ++i) {
            warm[nU - nu + i] = last_U_(nU - nu + i);
        }
    } else {
        for (int i = 0; i < nU; ++i) warm[i] = Uref(i);
    }
    osqp_warm_start_x(work, warm.data());

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
        last_U_ = U;
        have_warm_ = true;
    }

    result.force = U.segment(0, 3);

    Eigen::VectorXd x = x0;
    for (int k = 0; k < N; ++k) {
        Eigen::Vector3d uk = U.segment(k * nu, 3);
        x = A * x + B * uk + c;
        Eigen::Vector3d pos_i(x(0) + pad_x, x(1) + pad_y, std::max(0.0, x(2)));
        result.plan_pos.push_back(pos_i);
        if (x(2) <= 0.0) break;
    }

    osqp_cleanup(work);
    return result;
}

}  // namespace icarus
