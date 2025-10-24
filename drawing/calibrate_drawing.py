import json
import os
import queue
import select
import sys
import termios
import threading
import tty

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotProcessorPipeline, RobotAction
from lerobot.processor.converters import robot_action_to_transition, transition_to_robot_action
from lerobot.robots.so100_follower.robot_kinematic_processor import ForwardKinematicsJointsToEE
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader

key_queue = queue.Queue()

def read_keys():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # raw/cbreak mode
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                key_queue.put(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

threading.Thread(target=read_keys, daemon=True).start()

leader_port = '/dev/tty.usbmodem5A460826291'
leader_id = "my_leader"
follower_port = '/dev/tty.usbmodem5A460840631'
follower_id = "my_follower"

def connect_to_robots() -> tuple[SO101Leader, SO101Follower]:

    robot_config = SO101FollowerConfig(
        port=follower_port,
        id=follower_id,
    )

    teleop_config = SO101LeaderConfig(
        port=leader_port,
        id=leader_id
    )

    robot = SO101Follower(robot_config)
    teleop = SO101Leader(teleop_config)

    robot.connect()
    # robot.calibrate()

    teleop.connect()
    # teleop.calibrate()

    return robot, teleop

print("=== Drawing Calibration Tool ===")
print("Collect poses to build a drawing configuration.")
print("Controls:")
print("  Space - save current pose")
print("  Enter - finish and write config")
print()


robot, teleop = connect_to_robots()

kinematics_solver = RobotKinematics(
    urdf_path="./simulation/SO101/so101_new_calib.urdf",
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

poses = []

try:
    while True:
        action = teleop.get_action()
        robot.send_action(action)


        try:
            key = key_queue.get_nowait()
            print(f"Pressed: {key}")
            if key == "\r" or key == "q":
                break
            if key == " ":
                # Build pipeline to convert teleop joints to EE action
                # teleop joints -> teleop EE action
                leader_ee_act = leader_to_ee(action)
                poses.append(leader_ee_act)
                print("Saving pose")
        except queue.Empty:
            pass
except KeyboardInterrupt:
    pass
finally:

    robot.disconnect()
    teleop.disconnect()

    config = dict()
    config["robot_port"] = robot.config.port
    config["robot_id"] = robot.config.id
    config["robot_config_dir"] = robot.config.calibration_dir
    config["teleop_port"] = teleop.config.port
    config["teleop_id"] = teleop.config.id
    config["teleop_config_dir"] = teleop.config.calibration_dir
    config["poses"] = poses

    config_path = os.path.join(os.path.dirname(__file__), "drawing_config.json")
    #write config to file
    with open(config_path, "w") as f:
        f.write(json.dumps(config, indent=2, default=str))