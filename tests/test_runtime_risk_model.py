import unittest

from runtime_risk_model import ContextAwareRuntimeRiskModel, RobotState, SensorReading, SensorSetup


class RuntimeRiskModelTests(unittest.TestCase):
    def test_velocity_increases_risk(self) -> None:
        model = ContextAwareRuntimeRiskModel(sensor_setup=SensorSetup(smoothing_window=1))
        sensors = SensorReading(lidar_distance_m=10.0, obstacle_density=0.3, terrain_slip_index=0.1)

        low_speed = model.assess_risk(RobotState(velocity_mps=0.5), sensors)
        high_speed = model.assess_risk(RobotState(velocity_mps=2.8), sensors)

        self.assertGreater(high_speed["risk_score"], low_speed["risk_score"])

    def test_context_awareness_raises_risk(self) -> None:
        model = ContextAwareRuntimeRiskModel(sensor_setup=SensorSetup(smoothing_window=1))
        state = RobotState(velocity_mps=1.2)
        sensors = SensorReading(lidar_distance_m=8.0, obstacle_density=0.2, terrain_slip_index=0.1)

        nominal = model.assess_risk(state, sensors, context="nominal")
        offroad_low_visibility = model.assess_risk(state, sensors, context="offroad_low_visibility")

        self.assertGreater(offroad_low_visibility["risk_score"], nominal["risk_score"])

    def test_lidar_required_for_assessment(self) -> None:
        model = ContextAwareRuntimeRiskModel(sensor_setup=SensorSetup(lidar_enabled=False))

        with self.assertRaisesRegex(ValueError, "LiDAR must be enabled"):
            model.assess_risk(
                RobotState(velocity_mps=1.0),
                SensorReading(lidar_distance_m=6.0, obstacle_density=0.2),
            )

    def test_realtime_sensor_setup_uses_no_smoothing(self) -> None:
        setup = SensorSetup(smoothing_window=5, lidar_refresh_hz=15.0)
        setup.optimize_for_realtime()
        self.assertEqual(setup.smoothing_window, 1)
        self.assertGreaterEqual(setup.lidar_refresh_hz, 40.0)


if __name__ == "__main__":
    unittest.main()
