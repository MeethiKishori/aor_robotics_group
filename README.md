# aor_robotics_group
adaptive autonomy and off-road robotics group

## Runtime risk modeling (robot dog)

This repository now includes a context-aware runtime risk model in:

- `/home/runner/work/aor_robotics_group/aor_robotics_group/runtime_risk_model.py`

Key capabilities:

- Velocity-based risk assessment
- Context-aware risk multipliers (off-road, low-visibility)
- LiDAR-centric sensor setup checks
- Realtime-oriented sensor tuning (`optimize_for_realtime`)

### Run tests

```bash
cd /home/runner/work/aor_robotics_group/aor_robotics_group
python -m unittest discover -v
```
