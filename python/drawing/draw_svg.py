import numpy as np
from svgpathtools import svg2paths
import os
import json

from drawing.draw_points import draw_points


def load_svg_points():
    paths, attributes = svg2paths('svg/hello.svg')
    path = paths[0]
    num_samples = 200
    points = []
    for i in np.linspace(0, 1, num_samples):
        point = path.point(i)  # returns a complex number (x + yj)
        points.append([point.real, point.imag])
    points = np.array(points)
    return points


points = load_svg_points()
#flip Y
points[:, 1] = -points[:, 1]

config_path = os.path.join(os.path.dirname(__file__), "drawing_config.json")
with open(config_path) as f:
    config = json.load(f)

poses = [[pose["ee.x"], pose["ee.y"], pose["ee.z"]] for pose in config["poses"]]
poses = np.array(poses)


#map poses to 2D space
T, u, v, normal, origin = np.array(config["T"]), np.array(config["u"]), np.array(config["v"]), np.array(config["normal"]), np.array(config["origin"])
projected_poses_2d = []
for pose in poses:
    vec = pose - origin
    print(vec)
    s = np.dot(vec, u)
    t = np.dot(vec, v)
    projected_poses_2d.append([s, t])
projected_poses_2d = np.array(projected_poses_2d)

#get bounding box of projected poses
min_s, min_t = np.min(projected_poses_2d, axis=0)
max_s, max_t = np.max(projected_poses_2d, axis=0)
scale_s = (max_s - min_s) / (np.max(points[:,0]) - np.min(points[:,0]))
scale_t = (max_t - min_t) / (np.max(points[:,1]) - np.min(points[:,1]))
scale = min(scale_s, scale_t)

scaled_points = points * scale
#translate to fit in center of bounding box
scaled_points[:,0] += (min_s + max_s)/2 - (np.min(scaled_points[:,0]) + np.max(scaled_points[:,0]))/2
scaled_points[:,1] += (min_t + max_t)/2 - (np.min(scaled_points[:,1]) + np.max(scaled_points[:,1]))/2

#rotate points to match orientation of poses (u,v)
angle_u = np.arctan2(u[1], u[0])
rotation_matrix = np.array([[np.cos(angle_u), -np.sin(angle_u)],
                             [np.sin(angle_u),  np.cos(angle_u)]])
scaled_points = scaled_points @ rotation_matrix.T

try:
    draw_points(scaled_points)
except Exception as e:
    import matplotlib.pyplot as plt
    plt.scatter(*projected_poses_2d.T, label="EE area")
    plt.scatter(*scaled_points.T, label="Scaled SVG")
    plt.axis("equal")
    plt.legend()
    plt.show()

