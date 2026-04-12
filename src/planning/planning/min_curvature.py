import numpy as np
from scipy import sparse
from scipy.optimize import minimize, Bounds

from .centerline import (
    extract_centerline,
    compute_normals,
)


def build_qp(centerline, normals):
    """Build the sparse QP matrices for minimum curvature optimization.

    Minimizes sum of ||p_{i+1} - 2*p_i + p_{i-1}||^2 where
    p_i = centerline_i + alpha_i * normal_i.

    Args:
        centerline: Nx2 array of centerline waypoints.
        normals: Nx2 unit normal vectors at each waypoint.

    Returns:
        (M, b) where:
            M: sparse (2N x N) matrix
            b: (2N,) constant vector
        Objective is ||M @ alpha + b||^2
    """
    N = len(centerline)
    M = sparse.lil_matrix((2 * N, N))
    b = np.zeros(2 * N)

    for i in range(N):
        im = (i - 1) % N
        ip = (i + 1) % N

        # Constant term: q_i = c_{i+1} - 2*c_i + c_{i-1}
        q = centerline[ip] - 2.0 * centerline[i] + centerline[im]
        b[2 * i] = q[0]
        b[2 * i + 1] = q[1]

        # x-component row
        M[2 * i, im] = normals[im, 0]
        M[2 * i, i] = -2.0 * normals[i, 0]
        M[2 * i, ip] = normals[ip, 0]

        # y-component row
        M[2 * i + 1, im] = normals[im, 1]
        M[2 * i + 1, i] = -2.0 * normals[i, 1]
        M[2 * i + 1, ip] = normals[ip, 1]

    M = M.tocsr()
    return M, b


def optimize_min_curvature(centerline, normals, w_left, w_right, margin=0.155):
    """Solve the minimum curvature QP via L-BFGS-B.

    Args:
        centerline: Nx2 centerline waypoints.
        normals: Nx2 unit normals.
        w_left: N distances to left wall (meters).
        w_right: N distances to right wall (meters).
        margin: half vehicle width safety margin (meters).

    Returns:
        alpha_opt: N optimal lateral offsets.
    """
    N = len(centerline)
    M, b = build_qp(centerline, normals)

    # Precompute M^T for gradient
    Mt = M.T.tocsr()

    def objective(alpha):
        r = M @ alpha + b
        return 0.5 * np.dot(r, r)

    def gradient(alpha):
        r = M @ alpha + b
        return Mt @ r

    # Box constraints: stay within track boundaries minus margin
    lb = np.maximum(-w_right + margin, -w_right * 0.9)
    ub = np.minimum(w_left - margin, w_left * 0.9)

    # Ensure feasibility: if lb > ub at any point, clamp to centerline
    infeasible = lb > ub
    lb[infeasible] = 0.0
    ub[infeasible] = 0.0

    bounds = Bounds(lb, ub)
    alpha0 = np.zeros(N)

    result = minimize(
        objective,
        alpha0,
        jac=gradient,
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 500, 'ftol': 1e-12, 'gtol': 1e-8},
    )

    return result.x


def generate_raceline(map_yaml_path, spawn_x=0.0, spawn_y=0.0, spawn_theta=0.0,
                      vehicle_width=0.31, spacing=0.2):
    """Full pipeline: map -> minimum curvature racing line.

    Args:
        map_yaml_path: path to the map YAML file.
        spawn_x, spawn_y: spawn position for centerline ordering.
        spawn_theta: spawn heading (unused, for future direction ordering).
        vehicle_width: vehicle width in meters.
        spacing: waypoint spacing in meters.

    Returns:
        raceline: Nx2 optimized racing line waypoints.
        centerline: Nx2 original centerline.
        w_left, w_right: track width arrays.
    """
    centerline, w_left, w_right, free_mask, resolution, origin, img_height = (
        extract_centerline(map_yaml_path, spawn_x, spawn_y, spacing)
    )

    normals = compute_normals(centerline)
    margin = vehicle_width / 2.0

    alpha_opt = optimize_min_curvature(centerline, normals, w_left, w_right, margin)

    raceline = centerline + alpha_opt[:, np.newaxis] * normals

    return raceline, centerline, w_left, w_right
