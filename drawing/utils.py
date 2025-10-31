import time
import platform

import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R


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


def init_keyboard():
    events = {}
    events["record_pose"] = False
    events["stop_recording"] = False
    events["record_home"] = False

    def on_press(event):
        if hasattr(event, "char") and event.char is not None:
            print(
                f"You pressed {event.char} on the keyboard."
            )
            if event.char == "q":
                events["stop_recording"] = True
        else:
            if event.name == "space":
                events["record_pose"] = True
            elif event.name == "enter" or event.name == "escape" or event.name == "backspace" or event.name == "shift":
                events["stop_recording"] = True
            elif event.name == "home":
                events["record_home"] = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    return listener, events


def busy_wait(seconds):
    if platform.system() == "Darwin" or platform.system() == "Windows":
        # On Mac and Windows, `time.sleep` is not accurate and we need to use this while loop trick,
        # but it consumes CPU cycles.
        end_time = time.perf_counter() + seconds
        while time.perf_counter() < end_time:
            pass
    else:
        # On Linux time.sleep is accurate
        if seconds > 0:
            time.sleep(seconds)


def compose_ee_pose(pos_w, tilt_angle_deg, plane_u, plane_v, plane_normal, gripper_pos=8.5):
    """
    Compose a robot EE pose from a position on the drawing plane and a tilt angle.

    Parameters
    ----------
    pos_w : array-like, shape (3,)
        Position of end-effector in world coordinates.
    tilt_angle_deg : float
        Tilt angle of pencil relative to plane normal (degrees).
    plane_u, plane_v, plane_normal : np.ndarray, shape (3,)
        Orthonormal basis vectors of the plane.
    gripper_pos : float
        Optional gripper position value.

    Returns
    -------
    dict
        Dictionary with ee.{x,y,z,wx,wy,wz,gripper_pos}
    """
    # Normalize plane axes
    u = plane_u / np.linalg.norm(plane_u)
    v = plane_v / np.linalg.norm(plane_v)
    n = plane_normal / np.linalg.norm(plane_normal)

    # Step 1 — Apply tilt around plane_u
    tilt_axis = u  # could choose v or any axis in plane
    tilt_rad = np.radians(tilt_angle_deg)
    R_tilt = R.from_rotvec(tilt_rad * tilt_axis)

    # Step 2 — Build base orientation (pencil along -normal)
    z_axis = -n
    x_axis = u
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    R_base = np.column_stack([x_axis, y_axis, z_axis])

    # Step 3 — Apply tilt
    R_final = R_base #R_tilt.as_matrix() @ R_base

    # Step 4 — Convert to Euler angles (XYZ convention)
    euler = R.from_matrix(R_final).as_euler('xyz', degrees=False)

    return {
        "ee.x": float(pos_w[0]),
        "ee.y": float(pos_w[1]),
        "ee.z": float(pos_w[2]),
        "ee.wx": float(euler[0]),
        "ee.wy": float(euler[1]),
        "ee.wz": float(euler[2]),
        "ee.gripper_pos": float(gripper_pos)
    }
