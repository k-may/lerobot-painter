from typing import Any
import json
from lerobot.robots import Robot

class RobotWebSocket(Robot):
    def __init__(self, ip="localhost", port="8765"):
        self.ip = ip
        self.port = port

    def disconnect(self) -> None:
        self.ws.close()

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        self.ws.send(json.dumps(action))

    def connect(self):
        import websocket
        self.ws = websocket.WebSocket()
        self.ws.connect(f"ws://{self.ip}:{self.port}")
