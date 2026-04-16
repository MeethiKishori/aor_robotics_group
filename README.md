# aor_robotics_group
adaptive autonomy and off-road robotics group

## Runtime risk modeling (robot dog)

This repository now includes a context-aware runtime risk model in:

- `runtime_risk_model.py`

Key capabilities:

- Velocity-based risk assessment
- Context-aware risk multipliers (off-road, low-visibility)
- LiDAR-centric sensor setup checks
- Realtime-oriented sensor tuning (`optimize_for_realtime`)

### Run tests

```bash
python -m unittest discover -v
```
