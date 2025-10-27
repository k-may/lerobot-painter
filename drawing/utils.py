import numpy as np
from pynput import keyboard


def fit_plane_svd(points):
    # points: (N,3) numpy array in robot/world frame
    centroid = points.mean(axis=0)
    Q = points - centroid
    # SVD
    _, _, vh = np.linalg.svd(Q, full_matrices=False)
    normal = vh[-1, :]           # smallest singular vector
    normal = normal / np.linalg.norm(normal)
    # choose u axis: vector from first to second point projected onto plane
    tmp = points[1] - points[0]
    u = tmp - np.dot(tmp, normal) * normal
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    T = np.eye(4)
    T[0:3,0] = u
    T[0:3,1] = v
    T[0:3,2] = normal
    T[0:3,3] = centroid
    return T, u, v, normal, centroid

def init_keyboard():
    events = {}
    events["record_pose"] = False
    events["stop_recording"] = False

    def on_press(event):
        if event.name == "space":
            events["record_pose"] = True
        elif event.name == "q" or event.name == "enter":
            events["stop_recording"] = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    return listener, events