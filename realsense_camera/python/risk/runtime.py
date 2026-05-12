import time


class RiskRuntimeState:
    """Keeps runtime state for speed estimation and target closure tracking."""

    def __init__(self):
        self.speed_est = 0.0
        self.accel_mag = 0.0
        self.prev_target_distance = float("inf")
        self.last_t = time.time()

    def next_dt(self):
        now = time.time()
        dt = max(now - self.last_t, 1e-3)
        self.last_t = now
        return dt

    def update_speed(self, use_imu, dt):
        if not use_imu:
            self.speed_est = 0.0
            return self.speed_est

        # Remove gravity, integrate acceleration to estimate speed.
        lin_accel = max(0.0, self.accel_mag - 9.81)
        self.speed_est += lin_accel * dt
        self.speed_est *= 0.98
        self.speed_est = max(0.0, min(5.0, self.speed_est))
        return self.speed_est

    def compute_closure_rate(self, current_distance, dt):
        if current_distance < float("inf") and self.prev_target_distance < float("inf"):
            closure = max(0.0, (self.prev_target_distance - current_distance) / dt)
        else:
            closure = 0.0
        self.prev_target_distance = current_distance
        return closure
