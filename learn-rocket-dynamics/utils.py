import numpy as np
from typing import Optional

# Quaternion operations

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of a quaternion [w, x, y, z] -> [w, -x, -y, -z]."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize a quaternion to unit length."""
    norm = np.linalg.norm(q)
    return q / norm if norm > 0 else q

def quat_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector v by a quaternion q."""
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    rotated = quat_multiply(quat_multiply(q, v_quat), quat_conjugate(q))
    return rotated[1:]  # Return the vector part

def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert a quaternion [w, x, y, z] to 3x3 rotation matrix."""
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),      2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),      2*(y*z - w*x)],
        [    2*(x*z - y*w),     2*(y*z + w*x),  1 - 2*(x*x + y*y)],
    ])

def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Euler angles (ZYX convention, radians) -> quaternion [w, x, y, z]."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ])

def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [w, x, y, z] -> Euler angles (ZYX convention, radians)."""
    w, x, y, z = quat_normalize(q)
    sinp = 2 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)  # Clamp to avoid numerical issues
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return np.array([roll, pitch, yaw])

def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Axis-angle -> unit quaternion. Axis need not be normalized."""
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    half_angle = angle / 2
    return np.array([np.cos(half_angle), *(np.sin(half_angle) * axis)])

def quat_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """Quaternion time-derivative: q_dot = 0.5 * q x [0, omega]."""
    omega_quat = np.array([0.0, *omega])
    return 0.5 * quat_multiply(q, omega_quat)


def euler_angle_rates(omega: np.ndarray, roll: float, pitch: float) -> np.ndarray:
    """Body rates ω = (p, q, r) to ZYX Euler rates (φ̇, θ̇, ψ̇).

    Singular at pitch = ±π/2: the tan(θ) and 1/cos(θ) terms diverge.
    """
    p, q, r = omega
    sin_roll = np.sin(roll)
    cos_roll = np.cos(roll)
    cos_pitch = np.cos(pitch)
    tan_pitch = np.tan(pitch)

    roll_dot = p + (q * sin_roll + r * cos_roll) * tan_pitch
    pitch_dot = q * cos_roll - r * sin_roll
    yaw_dot = (q * sin_roll + r * cos_roll) / cos_pitch
    return np.array([roll_dot, pitch_dot, yaw_dot])

def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = np.dot(q0, q1)
    
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return quat_normalize(result)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta_t = theta_0 * t
    sin_theta_t = np.sin(theta_t)

    s0 = np.cos(theta_t) - dot * sin_theta_t / sin_theta_0
    s1 = sin_theta_t / sin_theta_0

    return s0 * q0 + s1 * q1

# Rotation matrix utilities

def rot_x(angle: float) -> np.ndarray:
    """Rotation matrix for rotation about X-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rot_y(angle: float) -> np.ndarray:
    """Rotation matrix for rotation about Y-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rot_z(angle: float) -> np.ndarray:
    """Rotation matrix for rotation about Z-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler angles -> rotation matrix."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


# Full trajectory integrators

def integrate_euler(deriv, x0, t_span, dt):
    """Forward Euler integration. Simple but energy-drifting."""
    t = np.arange(t_span[0], t_span[1], dt)
    x = np.zeros((len(t), len(x0)))
    x[0] = x0
    for i in range(len(t)-1):
        x[i+1] = x[i] + dt * deriv(t[i], x[i])
    return t, x


def integrate_rk4(deriv, x0, t_span, dt):
    """Classic 4th-order Runge-Kutta. The standard for a reason."""
    t = np.arange(t_span[0], t_span[1], dt)
    x = np.zeros((len(t), len(x0)))
    x[0] = x0
    for i in range(len(t)-1):
        k1 = deriv(t[i],             x[i])
        k2 = deriv(t[i] + 0.5 * dt, x[i] + 0.5 * dt * k1)
        k3 = deriv(t[i] + 0.5 * dt, x[i] + 0.5 * dt * k2)
        k4 = deriv(t[i] + dt,        x[i] + dt * k3)
        x[i+1] = x[i] + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return t, x


# 3D body drawing

def make_rocket_body(length: float = 2.0, radius: float = 0.2) -> np.ndarray:
    """Creates wireframe vertices for a simple rocket shape (body-frame).
    
    Returns (N, 3) array of points. The rocket points along +X in body frame,
    with the nose at +X and the engine at -X. Cross-section is in the YZ plane.
    """
    n_circ = 12
    theta = np.linspace(0, 2*np.pi, n_circ, endpoint=False)

    engine_x = -length / 2
    top_x = length / 2
    nose_x = top_x + length * 0.15

    pts = []
    for th in theta:
        y, z = radius * np.cos(th), radius * np.sin(th)
        pts.append([engine_x, y, z])
        pts.append([top_x, y, z])

    for th in theta:
        pts.append([top_x, radius * np.cos(th), radius * np.sin(th)])
        pts.append([nose_x, 0, 0])

    n_fin = 4
    fin_span = radius * 2.5
    for i in range(n_fin):
        th = i * 2 * np.pi / n_fin
        dy, dz = np.cos(th), np.sin(th)
        pts.append([engine_x, radius * dy, radius * dz])
        pts.append([engine_x - length * 0.08, fin_span * dy, fin_span * dz])
        pts.append([engine_x - length * 0.08, fin_span * dy, fin_span * dz])
        pts.append([engine_x + length * 0.12, radius * dy, radius * dz])

    return np.array(pts)


def draw_body_3d(ax, vertices: np.ndarray, R: np.ndarray, pos: Optional[np.ndarray] = None, color='steelblue', alpha=0.8, linewidth=1.5):
    """Draw a wireframe body on a 3D axes.

    vertices: (N, 3) pairs of line segment endpoints (every 2 rows = 1 line)
    R: 3x3 rotation matrix (body -> world)
    pos: (3,) translation)
    """
    if pos is None:
        pos = np.zeros(3)
    rotated = (R @ vertices.T).T + pos
    lines = []
    for i in range(0, len(rotated), 2):
        line, = ax.plot3D(
            [rotated[i, 0], rotated[i+1, 0]],
            [rotated[i, 1], rotated[i+1, 1]],
            [rotated[i, 2], rotated[i+1, 2]],
            color=color, alpha=alpha, linewidth=linewidth
        )
        lines.append(line)
    return lines

def draw_axes_3d(ax, R: np.ndarray, pos: Optional[np.ndarray] = None, length: float = 1.0, linewidth: float = 2.0):
    """Draw body-frame axes (RGB = XYZ) on a 3D axes."""
    if pos is None:
        pos = np.zeros(3)
    colors = ['red', 'green', 'blue']
    arrows = []
    for i in range(3):
        direction = R[:, i] * length
        arrow = ax.quiver(
            pos[0], pos[1], pos[2],
            direction[0], direction[1], direction[2],
            color=colors[i], linewidth=linewidth, 
            arrow_length_ratio=0.1
        )
        arrows.append(arrow)
    return arrows

def set_equal_3d_axes(ax, center: Optional[np.ndarray] = None, radius: float = 2.0):
    """Set equal aspect ratio for 3D axes."""
    if center is None:
        center = np.zeros(3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')