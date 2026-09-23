import json
import time
import serial
from .messages import Telemetry, MotionCommand

class RoverSerial:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2.0)

    def send_command(self, cmd: MotionCommand) -> None:
        payload = {"type":"cmd", "v_mps":float(cmd.linear_mps), "w_rps":float(cmd.angular_rps)}
        self.ser.write((json.dumps(payload) + "\n").encode())

    def stop(self) -> None:
        self.send_command(MotionCommand(0.0, 0.0, "stop"))

    def read_telemetry(self):
        line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return None
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return None
        if data.get("type") != "telemetry":
            return None
        return Telemetry(
            left_ticks=int(data.get("left_ticks",0)),
            right_ticks=int(data.get("right_ticks",0)),
            left_rpm=float(data.get("left_rpm",0.0)),
            right_rpm=float(data.get("right_rpm",0.0)),
            yaw_rate_rps=float(data.get("yaw_rate_rps",0.0)),
            obstacle_distance_m=data.get("obstacle_distance_m"),
            battery_v=data.get("battery_v")
        )
