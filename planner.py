from .messages import MotionCommand

class LocalPlanner:
    def __init__(self, max_v, max_w):
        self.max_v = max_v
        self.max_w = max_w

    def plan(self, perception):
        if not perception.goal_detected:
            return MotionCommand(0.0, 0.0, "search_for_goal")
        w = max(-self.max_w, min(self.max_w, 1.8*perception.goal_bearing_rad))
        turn_factor = max(0.25, 1.0-min(1.0, abs(w)/self.max_w))
        return MotionCommand(self.max_v*turn_factor, w, "nominal_plan")
