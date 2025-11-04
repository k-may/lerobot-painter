import numpy as np
import json
import time

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotProcessorPipeline, RobotAction, RobotObservation
from lerobot.processor.converters import robot_action_observation_to_transition, transition_to_robot_action
from lerobot.robots.so100_follower.robot_kinematic_processor import EEBoundsAndSafety, InverseKinematicsEEToJoints

from drawing.connect import connect_to_robots
from drawing.pose_utils import fit_drawing_rotation
from drawing.utils import compose_ee_pose, busy_wait

FPS = 30

def draw_points(points : np.array):

    with open('./drawing_config.json', 'r') as f:
        config = json.load(f)

    T, u, v, normal, origin = np.array(config["T"]), np.array(config["u"]), np.array(config["v"]), np.array(
        config["normal"]), np.array(config["origin"])
    rot_vec = fit_drawing_rotation(config["poses"])

    hover_h = 0.04  # 2 cm above plane

    # compute world positions including hover start and end
    positions = []
    pos_start = origin + points[0][0] * u + points[0][1] * v - normal * hover_h
    positions.append(pos_start)

    for point in points:
        pos_w = origin + point[0] * u + point[1] * v
        positions.append(pos_w)

    pos_end = origin + points[-1][0] * u + points[-1][1] * v - normal * hover_h
    positions.append(pos_end)

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

            pos_w = positions[i]

            action = {
                "ee.x": float(pos_w[0]),
                "ee.y": float(pos_w[1]),
                "ee.z": float(pos_w[2]),
                "ee.wx": rot_vec[0],
                "ee.wy": rot_vec[1],
                "ee.wz": rot_vec[2],
                "ee.gripper_pos": 8.5
            }

            # Get robot observation
            robot_obs = robot.get_observation()
            # combine teleop EE action with robot observation for IK
            combined_input = (action, robot_obs)

            follower_joints_act = ee_to_follower_joints(combined_input)
            robot.send_action(follower_joints_act)

            busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

            if i < len(points) - 1:
                i += 1
            else:
                break