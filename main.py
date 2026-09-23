import json
import cv2
from .serial_link import RoverSerial
from .perception import CameraPerception
from .state_estimator import StateEstimator
from .planner import LocalPlanner
from .safety import SafetySupervisor

def load_config(path="config/rover_config.json"):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def main():
    cfg = load_config()
    cap = cv2.VideoCapture(cfg["camera_index"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera_width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera_height"])
    cap.set(cv2.CAP_PROP_FPS, cfg["camera_fps"])
    if not cap.isOpened():
        raise RuntimeError("Camera could not be opened")

    rover = RoverSerial(cfg["serial_port"], cfg["baudrate"])
    perception = CameraPerception(cfg)
    estimator = StateEstimator(cfg["wheel_radius_m"], cfg["wheel_track_m"])
    planner = LocalPlanner(cfg["max_linear_speed_mps"], cfg["max_angular_speed_rps"])
    safety = SafetySupervisor(cfg)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                rover.stop()
                raise RuntimeError("Camera frame acquisition failed")
            telem = rover.read_telemetry()
            obstacle_distance = None
            if telem:
                estimator.update(telem)
                obstacle_distance = telem.obstacle_distance_m
            p = perception.process(frame, obstacle_distance)
            raw_cmd = planner.plan(p)
            safe_cmd = safety.apply(raw_cmd, p)
            rover.send_command(safe_cmd)

            overlay = frame.copy()
            label = (f"goal={p.goal_detected} bearing={p.goal_bearing_rad:+.2f} "
                     f"conf={min(p.free_space_confidence,p.visual_motion_confidence):.2f} "
                     f"v={safe_cmd.linear_mps:.2f} w={safe_cmd.angular_rps:+.2f} {safe_cmd.reason}")
            cv2.putText(overlay, label, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
            cv2.imshow("DART Rover", overlay)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        rover.stop()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
