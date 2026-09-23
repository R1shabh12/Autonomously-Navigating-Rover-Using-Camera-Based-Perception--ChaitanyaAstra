from dataclasses import dataclass
from typing import Optional

@dataclass
class Telemetry:
    left_ticks: int = 0
    right_ticks: int = 0
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    yaw_rate_rps: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    obstacle_distance_m: Optional[float] = None
    battery_v: Optional[float] = None

@dataclass
class Perception:
    goal_detected: bool
    goal_bearing_rad: float
    free_space_confidence: float
    obstacle_distance_m: Optional[float]
    visual_motion_confidence: float

@dataclass
class MotionCommand:
    linear_mps: float
    angular_rps: float
    reason: str = "nominal"
