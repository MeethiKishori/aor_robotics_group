# Latest Work

## Done Now
- Modular perception in python/perception/red_black_roi.py
- Modular risk in python/risk/risk_score.py
- Launch script in python/launch/run_risk_roi_red_black.py
- Live ROI + red/black detection + box + distance + risk works

## Run
cd ~/Finroc/singh_files/aor_robotics_group/realsense_camera
source realsense_env/bin/activate
python3 python/launch/run_risk_roi_red_black.py

## Future Ideas
1. Add launch mode switch: ROI-only, detect-only, full-risk
2. Add live HSV and MIN_AREA sliders
3. Keep only nearest RED and nearest BLACK object
4. Smooth distance over time to reduce jitter
5. Publish risk and detections to Finroc ports
6. Save logs to CSV
7. Add left/center/right ROI risk

