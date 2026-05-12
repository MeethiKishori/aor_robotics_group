import time
import math


class RiskRuntimeState:
    """Keeps runtime state for speed estimation and target closure tracking."""

    def __init__(self):
        self.speed_est = 0.0
        self.accel_mag = 0.0
        self.prev_target_distance = float("inf")
        self.prev_object_distances = {}
        self.prev_object_measurements = {}
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

    def compute_object_closure_rate(self, object_key, current_distance, dt):
        prev = self.prev_object_distances.get(object_key, float("inf"))
        if current_distance < float("inf") and prev < float("inf"):
            closure = max(0.0, (prev - current_distance) / dt)
        else:
            closure = 0.0
        self.prev_object_distances[object_key] = current_distance
        return closure

    def keep_only_keys(self, keys):
        self.prev_object_distances = {
            k: v for k, v in self.prev_object_distances.items() if k in keys
        }
        self.prev_object_measurements = {
            k: v for k, v in self.prev_object_measurements.items() if k in keys
        }

    def compute_object_velocity_3d(self, object_key, cx, cy, distance_m, dt, fx, fy):
        """Estimate per-object 3D velocity from pixel motion + depth.

        vx, vy are derived from image-plane motion and intrinsics:
          vx ~= (du * Z / fx) / dt
          vy ~= (dv * Z / fy) / dt
        vz is depth-axis motion from distance change.
        """
        prev = self.prev_object_measurements.get(object_key)

        vx = 0.0
        vy = 0.0
        vz = 0.0
        speed = 0.0

        if (
            prev is not None
            and distance_m < float("inf")
            and prev[2] < float("inf")
            and dt > 0
            and fx > 1e-6
            and fy > 1e-6
        ):
            prev_cx, prev_cy, prev_dist = prev
            du = float(cx - prev_cx)
            dv = float(cy - prev_cy)

            # Convert pixel motion to meters/second using pinhole approximation.
            vx = (du * distance_m / fx) / dt
            vy = (dv * distance_m / fy) / dt

            # Positive vz means object moving toward camera.
            vz = (prev_dist - distance_m) / dt

            speed = math.sqrt(vx * vx + vy * vy + vz * vz)

        self.prev_object_measurements[object_key] = (float(cx), float(cy), float(distance_m))
        return speed, vx, vy, vz
