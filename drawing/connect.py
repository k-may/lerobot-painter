from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader


def connect_to_robots(config, force_callibrate=False):
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
    robot = SO101Follower(robot_config)
    teleop = SO101Leader(teleop_config)

    @contextmanager
    def _cm():
        robot.connect()
        if robot.calibration is None or force_callibrate:
            robot.calibrate()

        teleop.connect()
        if teleop.calibration is None or force_callibrate:
            teleop.calibrate()
        try:
            yield robot, teleop
        finally:
            print("Disconnecting")

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
