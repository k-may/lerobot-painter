from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader


def connect_to_robots(leader_port, leader_id, follower_port, follower_id, force_callibrate=False):
    from contextlib import contextmanager

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
