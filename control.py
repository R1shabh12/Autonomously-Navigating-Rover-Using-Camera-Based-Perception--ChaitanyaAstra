from .messages import MotionCommand

class DifferentialDriveController:
    def __init__(self, wheel_radius_m, track_m):
        self.r = wheel_radius_m
        self.L = track_m

    def wheel_angular_rates(self, cmd: MotionCommand):
        right = (cmd.linear_mps + cmd.angular_rps*self.L/2.0)/self.r
        left = (cmd.linear_mps - cmd.angular_rps*self.L/2.0)/self.r
        return left, right
