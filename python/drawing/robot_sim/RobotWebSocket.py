import json
import threading
from typing import Any

from lerobot.processor import RobotObservation
from lerobot.robots import Robot


class RobotSocketBus:
    def __init__(self, motors: dict[str, Any]) -> None:
        self.motors = motors

    def __getitem__(self, key: str) -> Any:
        return self.motors[key]

    def items(self):
        return self.motors.items()

    def set_joints(self, action: dict[str, Any]) -> None:
        for motor, val in action.items():
            self.motors[motor] = val

class RobotWebSocket(Robot):
    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        return self.observation_features

    @property
    def observation_features(self) -> dict:
        obs_dict = {f"{motor}.pos": val for motor, val in self.bus.items()}
        return obs_dict

    @property
    def action_features(self) -> dict:
        pass

    @property
    def is_connected(self) -> bool:
        return self.ws is not None and self._is_connected

    @property
    def is_calibrated(self) -> bool:
        pass

    def __init__(self, ip="172.18.224.1", port="8765", motors=None) -> None:
        self.ip = ip
        self.port = port
        self.ws = None
        self.bus = RobotSocketBus(motors)
        self._is_connected = False

    def disconnect(self) -> None:
        self.ws.close()

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        try:
            # print("Sending action:", action)
            self.ws.send(json.dumps(action))
            self.bus.set_joints(action)
        except Exception as e:
            self.bus.set_joints(action)
            # print(f"Error sending action: {e}")

    def connect(self):
        import websocket
        websocket.enableTrace(True)
        connected_event = threading.Event()
        def on_open(c):
            self._is_connected = True
            # self.ws.send("Hello")
            print("Connected")
            connected_event.set()

        self.ws = websocket.WebSocketApp(f"ws://{self.ip}:{self.port}", on_open=on_open)

        t = threading.Thread(target=self.ws.run_forever, daemon=True)
        t.start()
        connected_event.wait()
