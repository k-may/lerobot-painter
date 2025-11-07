import time
import platform

import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R





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


def compose_ee_pose(pos_w, plane_u, plane_v, plane_normal, gripper_pos=8.5):
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

    # Step 2 — Build base orientation (pencil along -normal)
    z_axis = -n
    x_axis = u
    y_axis = v #np.cross(z_axis, x_axis)
    # y_axis /= np.linalg.norm(y_axis)
    # x_axis = np.cross(y_axis, z_axis)
    # x_axis /= np.linalg.norm(x_axis)
    R_base = np.column_stack([x_axis, y_axis, z_axis])

    # Step 3 — Apply tilt
    R_final = R_base #R_tilt.as_matrix() @ R_base

    # # Step 4 — Convert to Euler angles (XYZ convention)
    rot_vec = R.from_matrix(R_final).as_rotvec()

    return {
        "ee.x": float(pos_w[0]),
        "ee.y": float(pos_w[1]),
        "ee.z": float(pos_w[2]),
        "ee.wx": float(rot_vec[0]),
        "ee.wy": float(rot_vec[1]),
        "ee.wz": float(rot_vec[2]),
        "ee.gripper_pos": float(gripper_pos)
    }


def align_plane_frame_z(rv_sample, R_plane):
    """
    Align plane frame R_plane to reference sample rotation rv_sample.
    Assumes z-axis is already aligned.
    Rotates R_plane around z to match sample x-axis as closely as possible.
    """
    R_samp = R.from_rotvec(rv_sample).as_matrix()

    # Current z-axis of plane frame (should already match sample z)
    z_plane = R_plane[:, 2]
    x_samp = R_samp[:, 0]  # sample x-axis
    # Project sample x onto plane perpendicular to z
    x_samp_proj = x_samp - np.dot(x_samp, z_plane) * z_plane
    x_samp_proj /= np.linalg.norm(x_samp_proj)

    # Current x-axis of plane frame projected onto same plane
    x_plane_proj = R_plane[:, 0] - np.dot(R_plane[:, 0], z_plane) * z_plane
    x_plane_proj /= np.linalg.norm(x_plane_proj)

    # Rotation around z to align projected x axes
    cos_theta = np.clip(np.dot(x_plane_proj, x_samp_proj), -1.0, 1.0)
    theta = np.arccos(cos_theta)
    # Determine sign using cross product
    if np.dot(np.cross(x_plane_proj, x_samp_proj), z_plane) < 0:
        theta = -theta

    # Apply rotation around z
    R_rot = R.from_rotvec(theta * z_plane)
    R_aligned = R_rot.as_matrix() @ R_plane
    return R_aligned