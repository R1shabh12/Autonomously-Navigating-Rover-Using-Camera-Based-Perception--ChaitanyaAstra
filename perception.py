import cv2
import numpy as np
from .messages import Perception

class CameraPerception:
    """Prototype camera-first perception.

    The goal is represented by a green visual target. The perception interface is
    deliberately modular so a trained free-space/obstacle model can replace the
    heuristics without changing the planner, safety, or control layers.
    """
    def __init__(self, cfg):
        self.cfg = cfg

    def detect_goal(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo = np.array(self.cfg["goal_color_lower_hsv"], dtype=np.uint8)
        hi = np.array(self.cfg["goal_color_upper_hsv"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.cfg["min_goal_area_px"]:
            return False, 0.0
        m = cv2.moments(c)
        if m["m00"] == 0:
            return False, 0.0
        cx = m["m10"] / m["m00"]
        width = frame.shape[1]
        x_norm = (cx - width/2.0) / (width/2.0)
        bearing = float(np.arctan(x_norm))
        confidence = float(np.clip(area / 8000.0, 0.0, 1.0))
        return True, bearing * max(confidence, 0.35)

    def estimate_free_space_confidence(self, frame):
        h, w = frame.shape[:2]
        roi = frame[int(h*0.55):int(h*0.95), int(w*0.15):int(w*0.85)]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        variance = float(np.var(gray))
        conf = 0.55 + 0.35*np.tanh((variance-200.0)/500.0)
        return float(np.clip(conf, 0.20, 0.90))

    def process(self, frame, obstacle_distance_m=None):
        goal_detected, goal_bearing = self.detect_goal(frame)
        return Perception(
            goal_detected=goal_detected,
            goal_bearing_rad=goal_bearing,
            free_space_confidence=self.estimate_free_space_confidence(frame),
            obstacle_distance_m=obstacle_distance_m,
            visual_motion_confidence=0.75 if goal_detected else 0.45
        )
