from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader

from drawing.robot_sim.RobotWebSocket import RobotWebSocket
from drawing.utils import busy_wait

def connect_to_robots(config : dict, force_callibrate=False):
    from contextlib import contextmanager

    leader_port = config["teleop_port"]
    leader_id = config["teleop_id"]
    follower_port = config["robot_port"]
    follower_id = config["robot_id"]

    robot_config = SO101FollowerConfig(
        port=follower_port,
        id=follower_id,
    )
    teleop_config = SO101LeaderConfig(
        port=leader_port,
        id=leader_id
    )
    try :
        robot = SO101Follower(robot_config)
        teleop = SO101Leader(teleop_config)
        robot.connect()
        if robot.calibration is None or force_callibrate:
            robot.calibrate()

        teleop.connect()
        if teleop.calibration is None or force_callibrate:
            teleop.calibrate()
    except ConnectionError as e:

        pose = config["home_pose"]
        motors = {motor.replace(".pos", ""): pose[motor] for motor in pose.keys()}
        robot = RobotWebSocket(ip='172.18.224.1', port='8765', motors=motors)
        robot.connect()

        teleop = None

    @contextmanager
    def _cm():

        try:
            yield robot, teleop
        finally:
            print("Disconnecting")

            if config["home_pose"] is not None and robot.is_connected:
                robot.send_action(config["home_pose"])
                busy_wait(2)

            try:
                if robot.is_connected:
                    robot.disconnect()
            except Exception:
                pass
            try:
                if teleop.is_connected:
                    teleop.disconnect()
            except Exception:
                pass

    return _cm()
