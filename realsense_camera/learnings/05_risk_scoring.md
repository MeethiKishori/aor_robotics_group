# Risk Scoring (0 to 10) using Depth + IMU

## Done — v1 Basic Prototype

- `risk.py` reads depth + IMU (accel) from D435i.
- Computes `d_front`: nearest obstacle in center 30% of depth image (10th percentile).
- Estimates camera speed from IMU accel integration (crude but functional).
- Tracks **closure rate**: how fast `d_front` shrinks per second.
  Detects moving obstacles even when the camera is completely still.
- `effective_speed = max(camera_speed, closure_rate)`
- Computes `TTC = d_front / effective_speed`
- Maps TTC + speed to a risk score 0–10.

### Risk Levels

| Score | Level  | Label    |
|-------|--------|----------|
| 0–3   | GREEN  | LOW      |
| 4–6   | YELLOW | MODERATE |
| 7–10  | RED    | DANGER   |

Live output: `[LEVEL] risk=X/10  dist=Xm  cam=Xm/s  closure=Xm/s  ttc=Xs`

## Future Improvements (in order)

1. Replace IMU-integrated speed with better odometry or VIO speed (IMU drift is an issue).
2. Add stability penalty: high gyro (fast turning) adds +1 or +2 to risk.
3. Tune TTC and speed thresholds after real driving tests on the robot.
4. Use multiple depth ROIs (left/right/center) instead of only center, so side obstacles are caught.
5. Publish risk value to Finroc port instead of just printing to terminal.
6. Add depth-based direction of danger (not just a single number).
