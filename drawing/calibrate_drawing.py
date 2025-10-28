import json
import os

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotProcessorPipeline, RobotAction
from lerobot.processor.converters import robot_action_to_transition, transition_to_robot_action
from lerobot.robots.so100_follower.robot_kinematic_processor import ForwardKinematicsJointsToEE

from drawing.connect import connect_to_robots
from drawing.utils import fit_plane_svd, init_keyboard

print("=== Drawing Calibration Tool ===")
print("Collect poses to build a drawing configuration.")
print("Controls:")
print("  Space - save current pose")
print("  Enter - finish and write config")
print()

leader_port = '/dev/tty.usbmodem5A460826291'
leader_id = "my_leader"
follower_port = '/dev/tty.usbmodem5A460840631'
follower_id = "my_follower"

home_action = None
poses = []
poses_already_defined = False

# load previous config if it exists
config_path = os.path.join(os.path.dirname(__file__), "drawing_config.json")
if os.path.exists(config_path):
    with open(config_path, "r") as f:
        config = json.load(f)
        home_action = config.get("home_pose", None)
        leader_port = config.get("robot_port", leader_port)
        leader_id = config.get("robot_id", leader_id)
        follower_port = config.get("teleop_port", follower_port)
        follower_id = config.get("teleop_id", follower_id)
        poses = config.get("poses", [])
        poses_already_defined = True
else:
    config = dict()
    config["robot_port"] = leader_port
    config["robot_id"] = leader_id
    config["teleop_port"] = follower_port
    config["teleop_id"] = follower_id

with connect_to_robots(config) as (robot, teleop):

    kinematics_solver = RobotKinematics(
        urdf_path="../simulation/SO101/so101_new_calib.urdf",
        target_frame_name="gripper_frame_link",
        joint_names=list(robot.bus.motors.keys()),
    )

    # Build pipeline to convert teleop joints to EE action
    leader_to_ee = RobotProcessorPipeline[RobotAction, RobotAction](
        steps=[
            ForwardKinematicsJointsToEE(
                kinematics=kinematics_solver, motor_names=list(robot.bus.motors.keys())
            ),
        ],
        to_transition=robot_action_to_transition,
        to_output=transition_to_robot_action,
    )

    _, events = init_keyboard()

    try:
        while True:
            action = teleop.get_action()
            robot.send_action(action)

            if events["record_pose"]:
                if poses_already_defined:
                    print("Resetting all poses")
                    poses = []
                    poses_already_defined = False

                # Build pipeline to convert teleop joints to EE action
                # teleop joints -> teleop EE action
                leader_ee_act = leader_to_ee(action)
                poses.append(leader_ee_act)
                print("Saving pose")
                events["record_pose"] = False

            if events["stop_recording"]:
                break

            if events["record_home"]:
                home_action = action
                events["record_home"] = False

    except KeyboardInterrupt:
        pass
    finally:

        if len(poses) > 0:
            points = np.array([[p["ee.x"], p["ee.y"], p["ee.z"]] for p in poses])
            T, u, v, normal, origin = fit_plane_svd(points)
            distances = (points - origin) @ normal
            rms = np.sqrt(np.mean(distances ** 2))

            config = dict()
            config["home_pose"] = home_action if home_action is not None else poses[0]
            config["robot_port"] = robot.config.port
            config["robot_id"] = robot.config.id
            config["robot_config_dir"] = robot.config.calibration_dir
            config["teleop_port"] = teleop.config.port
            config["teleop_id"] = teleop.config.id
            config["teleop_config_dir"] = teleop.config.calibration_dir
            config["poses"] = poses
            config["T"] = T.tolist()
            config["u"] = u.tolist()
            config["v"] = v.tolist()
            config["normal"] = normal.tolist()
            config["origin"] = origin.tolist()
            config["rms"] = float(rms)

            config_path = os.path.join(os.path.dirname(__file__), "drawing_config.json")
            # write config to file
            with open(config_path, "w") as f:
                f.write(json.dumps(config, indent=2, default=str))
        else:
            print("No poses collected, not writing config.")
