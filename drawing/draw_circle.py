import json
import time

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotProcessorPipeline, RobotAction, RobotObservation
from lerobot.processor.converters import robot_action_observation_to_transition, transition_to_robot_action
from lerobot.robots.so100_follower.robot_kinematic_processor import EEBoundsAndSafety, InverseKinematicsEEToJoints

from drawing.connect import connect_to_robots
from drawing.utils import busy_wait, compose_ee_pose, align_plane_frame_z
from scipy.spatial.transform import Rotation as R

FPS = 30

with open('./drawing_config.json', 'r') as f:
    config = json.load(f)

T, u, v, normal, origin = np.array(config["T"]), np.array(config["u"]), np.array(config["v"]), np.array(config["normal"]), np.array(config["origin"])

# create 2D circle path in meters, center (0,0) radius 0.05
angles = np.linspace(0, 2 * np.pi, 300)
circle2d = [(0.05 * np.cos(a), 0.05 * np.sin(a)) for a in angles]

# convert to world poses and execute
hover_h = 0.01  # 2 cm above plane
contact_z = 0.0  # exactly on plane, or small negative for slight pressure

z_axis = -normal  # flip
y_axis = np.cross(z_axis, u)
y_axis /= np.linalg.norm(y_axis)
x_axis = np.cross(y_axis, z_axis)
x_axis /= np.linalg.norm(x_axis)
R_base = np.column_stack([x_axis, y_axis, z_axis])

rotvec_sample = np.array([-0.5778150380753542, 2.620369318561434, 0.141891978245403])
R_aligned = align_plane_frame_z(rotvec_sample, R_base)

rotvec_aligned = R.from_matrix(R_aligned).as_rotvec()

with connect_to_robots(config) as (robot, teleop):

    kinematics_solver = RobotKinematics(
        urdf_path="../simulation/SO101/so101_new_calib.urdf",
        target_frame_name="gripper_frame_link",
        joint_names=list(robot.bus.motors.keys()),
    )

    # build pipeline to convert EE action to robot joints
    ee_to_follower_joints = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        [
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=0.10,
            ),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                initial_guess_current_joints=False,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    i = 0
    while True:

        t0 = time.perf_counter()

        s,t = circle2d[i]
        pos_w = origin + s * u + t * v + hover_h * normal

        # Get robot observation
        robot_obs = robot.get_observation()

        action = {
            "ee.x": float(pos_w[0]),
            "ee.y": float(pos_w[1]),
            "ee.z": float(pos_w[2]),
            "ee.wx": rotvec_aligned[0], #-0.5778150380753542,
            "ee.wy": rotvec_aligned[1], #2.620369318561434,
            "ee.wz": rotvec_aligned[2], #0.141891978245403,
            "ee.gripper_pos": 8.5
        }
        # action = compose_ee_pose(pos_w,  u, v, normal)

        # combine teleop EE action with robot observation for IK
        combined_input = (action, robot_obs)

        follower_joints_act = ee_to_follower_joints(combined_input)
        robot.send_action(follower_joints_act)
        busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

        if i < len(circle2d) - 1:
            i += 1
        else:
            break
