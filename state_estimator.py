import math
import time

class StateEstimator:
    def __init__(self, wheel_radius_m, track_m, ticks_per_rev=1024):
        self.r = wheel_radius_m
        self.L = track_m
        self.ticks_per_rev = ticks_per_rev
        self.x = self.y = self.theta = 0.0
        self.prev_left = self.prev_right = None
        self.last_t = time.monotonic()

    def update(self, telem):
        now = time.monotonic()
        _dt = max(1e-3, now-self.last_t)
        self.last_t = now
        if self.prev_left is None:
            self.prev_left, self.prev_right = telem.left_ticks, telem.right_ticks
            return
        dlt = telem.left_ticks-self.prev_left
        drt = telem.right_ticks-self.prev_right
        self.prev_left, self.prev_right = telem.left_ticks, telem.right_ticks
        dL = 2*math.pi*self.r*(dlt/self.ticks_per_rev)
        dR = 2*math.pi*self.r*(drt/self.ticks_per_rev)
        ds = 0.5*(dL+dR)
        self.theta += (dR-dL)/self.L
        self.x += ds*math.cos(self.theta)
        self.y += ds*math.sin(self.theta)

    @property
    def pose(self):
        return self.x, self.y, self.theta
