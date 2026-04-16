from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict


def _clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class RobotState:
    velocity_mps: float


@dataclass(frozen=True)
class SensorReading:
    lidar_distance_m: float
    obstacle_density: float
    terrain_slip_index: float = 0.0


@dataclass
class SensorSetup:
    lidar_enabled: bool = True
    lidar_range_m: float = 20.0
    lidar_refresh_hz: float = 20.0
    smoothing_window: int = 3

    def optimize_for_realtime(self) -> None:
        self.lidar_refresh_hz = max(self.lidar_refresh_hz, 40.0)
        self.smoothing_window = 1


@dataclass
class ContextAwareRuntimeRiskModel:
    sensor_setup: SensorSetup = field(default_factory=SensorSetup)
    max_safe_velocity_mps: float = 3.0
    velocity_weight: float = 0.35
    proximity_weight: float = 0.35
    obstacle_weight: float = 0.2
    slip_weight: float = 0.1
    _recent_scores: Deque[float] = field(default_factory=deque, init=False, repr=False)

    def assess_risk(
        self,
        state: RobotState,
        sensors: SensorReading,
        context: str = "nominal",
    ) -> Dict[str, float | str]:
        if not self.sensor_setup.lidar_enabled:
            raise ValueError("LiDAR must be enabled for runtime risk assessment.")
        if self.sensor_setup.lidar_range_m <= 0:
            raise ValueError("LiDAR range must be greater than zero.")

        proximity_risk = _clamp(1.0 - (sensors.lidar_distance_m / self.sensor_setup.lidar_range_m))
        velocity_risk = _clamp(abs(state.velocity_mps) / self.max_safe_velocity_mps)
        obstacle_risk = _clamp(sensors.obstacle_density)
        slip_risk = _clamp(sensors.terrain_slip_index)

        score = (
            velocity_risk * self.velocity_weight
            + proximity_risk * self.proximity_weight
            + obstacle_risk * self.obstacle_weight
            + slip_risk * self.slip_weight
        )

        context_multiplier = {
            "nominal": 1.0,
            "offroad": 1.1,
            "low_visibility": 1.2,
            "offroad_low_visibility": 1.3,
        }.get(context, 1.0)
        score = _clamp(score * context_multiplier)

        window = max(1, self.sensor_setup.smoothing_window)
        self._recent_scores.append(score)
        if len(self._recent_scores) > window:
            self._recent_scores.popleft()
        smoothed_score = sum(self._recent_scores) / len(self._recent_scores)

        if smoothed_score < 0.33:
            level = "low"
        elif smoothed_score < 0.66:
            level = "medium"
        else:
            level = "high"

        return {"risk_score": round(smoothed_score, 3), "risk_level": level}
