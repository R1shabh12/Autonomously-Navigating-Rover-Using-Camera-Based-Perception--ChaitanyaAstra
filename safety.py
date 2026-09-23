from .messages import MotionCommand

class SafetySupervisor:
    def __init__(self, cfg):
        self.min_conf = cfg["safety_confidence_min"]
        self.slow_conf = cfg["slow_confidence_threshold"]
        self.hard_stop = cfg["hard_stop_distance_m"]
        self.slow_distance = cfg["slow_distance_m"]

    def apply(self, cmd, perception):
        conf = min(perception.free_space_confidence, perception.visual_motion_confidence)
        d = perception.obstacle_distance_m
        if d is not None and d <= self.hard_stop:
            return MotionCommand(0, 0, "hard_stop_obstacle")
        if d is not None and d <= self.slow_distance:
            return MotionCommand(min(cmd.linear_mps, 0.15), cmd.angular_rps, "slow_for_obstacle")
        if conf < self.min_conf:
            return MotionCommand(0, 0, "stop_reobserve")
        if conf < self.slow_conf:
            return MotionCommand(cmd.linear_mps*0.45, cmd.angular_rps, "slow_low_confidence")
        return cmd
