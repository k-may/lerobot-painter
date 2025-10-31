# --- helpers ---
import math

import numpy as np


def euler_to_quat(roll, pitch, yaw):
    """Convert roll, pitch, yaw (xyz / intrinsic) to quaternion [w, x, y, z]."""
    cr = np.cos(roll/2); sr = np.sin(roll/2)
    cp = np.cos(pitch/2); sp = np.sin(pitch/2)
    cy = np.cos(yaw/2); sy = np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w, x, y, z])

def quat_to_rotmat(q):
    """Quaternion (w,x,y,z) to 3x3 rotation matrix."""
    w,x,y,z = q
    Nq = w*w + x*x + y*y + z*z
    if Nq < 1e-12:
        return np.eye(3)
    s = 2.0 / Nq
    xx, yy, zz = x*x*s/2, y*y*s/2, z*z*s/2
    xy, xz, yz = x*y*s/2, x*z*s/2, y*z*s/2
    wx, wy, wz = w*x*s/2, w*y*s/2, w*z*s/2
    R = np.array([
        [1 - (yy+zz)*2 + 0, 2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),        1 - (xx+zz)*2 + 0,2*(yz - wx)],
        [2*(xz - wy),        2*(yz + wx),       1 - (xx+yy)*2 + 0]
    ])
    # The formula above is simplified; use standard conversion:
    w,x,y,z = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w),   1-2*(x*x+y*y)]
    ])
    return R

def average_quaternions_markley(quats):
    """Markley method: return the normalized eigenvector of sum(q_i q_i^T)."""
    # quats: Nx4 array, each as [w,x,y,z]
    A = np.zeros((4,4))
    for q in quats:
        q = q.reshape(4,1)
        A += q @ q.T
    # eigenvector with largest eigenvalue:
    vals, vecs = np.linalg.eig(A)
    max_idx = np.argmax(vals.real)
    q_avg = vecs[:, max_idx].real
    # normalize
    q_avg = q_avg / np.linalg.norm(q_avg)
    # ensure scalar part positive (convention)
    if q_avg[0] < 0:
        q_avg = -q_avg
    return q_avg

def rot_to_euler(R):
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2(R[1,0], R[0,0])
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0
    return roll, pitch, yaw

def build_plane_oriented_pose(R_ref, n, flip_tool_toward_surface=True):
    # choose z-axis direction
    z_axis = -n if flip_tool_toward_surface else n
    # reference x-axis
    x_ref = R_ref[:, 0]
    # project reference x into the plane
    x_proj = x_ref - np.dot(x_ref, z_axis) * z_axis
    if np.linalg.norm(x_proj) < 1e-6:
        # fallback
        x_proj = np.cross([1,0,0], z_axis)
        if np.linalg.norm(x_proj) < 1e-6:
            x_proj = np.cross([0,1,0], z_axis)
    x_proj /= np.linalg.norm(x_proj)
    y_axis = np.cross(z_axis, x_proj)
    return np.column_stack([x_proj, y_axis, z_axis])

def plot_poses(T, u,v,origin, normal, poses):
    import plotly.graph_objects as go
    from scipy.spatial.transform import Rotation as R

    def arrow(start, vec, color, name="arrow"):
        return go.Scatter3d(
            x=[start[0], start[0] + vec[0]],
            y=[start[1], start[1] + vec[1]],
            z=[start[2], start[2] + vec[2]],
            mode="lines+markers",
            line=dict(color=color, width=2),
            marker=dict(size=3, color=color),
            name=name
        )

    fig = go.Figure()

    # Axes
    fig.add_trace(arrow(origin, u * 0.01, "red"))
    fig.add_trace(arrow(origin, v * 0.01, "green"))
    fig.add_trace(arrow(origin, normal * 0.01, "black"))

    # pose positions
    for i, s in enumerate(poses):
        pos = np.array([s["ee.x"], s["ee.y"], s["ee.z"]])
        rot = np.array([s["ee.wx"], s["ee.wy"], s["ee.wz"]])
        rot = R.from_euler('xyz', rot)
        R_matrix = rot.as_matrix()
        x_axis = R_matrix[:, 0]  # End-effector X axis in base frame
        y_axis = R_matrix[:, 1]  # End-effector Y axis
        z_axis = R_matrix[:, 2]  # End-effector Z axis

        line_color = f"rgb({i * 15 % 255}, {i * 30 % 255}, 255)"
        fig.add_trace(arrow(pos, z_axis * 0.01, 'blue', "z-axis"))
        # fig.add_trace(arrow(pos, x_axis * 0.01, "red"))
        # fig.add_trace(arrow(pos, y_axis * 0.01, "green"))
        # --- Define rotation of +20° (in radians) about local x-axis ---
        angle_deg = -35
        angle_rad = np.deg2rad(angle_deg)
        R_local = R.from_rotvec(y_axis * angle_rad)  # rotate around local x-axis

        # --- Apply this local rotation to z-axis ---
        z_axis_rotated = R_local.apply(z_axis)

        # move along x_axis by 0.02m
        pos = pos + x_axis * 0.03
        pos = pos + z_axis * 0.07  # lift a bit
        fig.add_trace(arrow(pos, z_axis_rotated * 0.05, line_color))

        fig.add_trace(
            go.Scatter3d(
                x=[pos[0]],
                y=[pos[1]],
                z=[pos[2]],
                mode="markers",
                marker=dict(size=4, color="blue"),
            )
        )

    # Layout
    r = 0.12
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[origin[0] - r, origin[0] + r], title='X'),
            yaxis=dict(range=[origin[1] - r, origin[1] + r], title='Y'),
            zaxis=dict(range=[origin[2] - r, origin[2] + r], title='Z'),
            aspectmode='cube'
        ),
        margin=dict(l=0, r=0, t=40, b=0),
        title="Interactive TCP Visualization",
    )
    fig.show()