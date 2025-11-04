# --- helpers ---

import numpy as np


def plot_poses(T, u,v,origin, normal, poses):
    import plotly.graph_objects as go
    from scipy.spatial.transform import Rotation as R

    def arrow(start, vec, color, name=""):
        return go.Scatter3d(
            x=[start[0], start[0] + vec[0]],
            y=[start[1], start[1] + vec[1]],
            z=[start[2], start[2] + vec[2]],
            mode="lines+markers",
            line=dict(color=color, width=4),
            marker=dict(size=3, color=color),
            name=name
        )

    fig = go.Figure()

    # Axes
    fig.add_trace(arrow(origin, u * 0.01, "red", "u"))
    fig.add_trace(arrow(origin, v * 0.01, "green", "v"))
    fig.add_trace(arrow(origin, normal * 0.01, "black", "normal"))

    # pose positions
    for i, s in enumerate(poses):
        pos = np.array([s["ee.x"], s["ee.y"], s["ee.z"]])
        rotvec = np.array([s["ee.wx"], s["ee.wy"], s["ee.wz"]])

        # --- Convert to rotation matrix ---
        rot = R.from_rotvec(rotvec)
        R_mat = rot.as_matrix()

        # --- Original coordinate axes ---
        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
        z_axis = np.array([0, 0, 1])

        # --- Rotated axes ---
        x_rot = R_mat @ x_axis
        y_rot = R_mat @ y_axis
        z_rot = R_mat @ z_axis

        fig.add_trace(arrow(pos, x_rot * 0.01, 'red'))
        fig.add_trace(arrow(pos, y_rot * 0.01, 'green'))
        fig.add_trace(arrow(pos, z_rot * 0.01, 'black'))

    # Layout
    r = 1
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

def fit_plane_svd(points):
    # points: (N,3) numpy array in robot/world frame
    centroid = points.mean(axis=0)
    Q = points - centroid
    # SVD
    _, _, vh = np.linalg.svd(Q, full_matrices=False)
    normal = vh[-1, :]  # smallest singular vector
    normal = normal / np.linalg.norm(normal)
    # choose u axis: vector from first to second point projected onto plane
    tmp = points[1] - points[0]
    u = tmp - np.dot(tmp, normal) * normal
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    T = np.eye(4)
    T[0:3, 0] = u
    T[0:3, 1] = v
    T[0:3, 2] = normal
    T[0:3, 3] = centroid
    return T, u, v, normal, centroid

def fit_drawing_rotation(poses):
    from scipy.spatial.transform import Rotation as R

    rotvecs = np.array([[p["ee.wx"], p["ee.wy"], p["ee.wz"]] for p in poses])
    R_mats = R.from_rotvec(rotvecs).as_matrix()

    # Average rotation matrix
    R_avg = np.mean(R_mats, axis=0)

    # Re-orthogonalize using SVD
    U, _, Vt = np.linalg.svd(R_avg)
    R_ortho = U @ Vt

    rotvec_avg = R.from_matrix(R_ortho).as_rotvec()
    return rotvec_avg