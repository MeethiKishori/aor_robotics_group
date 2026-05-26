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







In launch file updates
im trying to achieve : capture black and red objects, get ttc


When an object moves laterally (sideways), radial velocity (depth change) alone won't give you the true speed. You need to account for movement across the pixel plane.
    1. Vector Summation (2D + Depth)
    2. Optical Flow (Lucas-Kanade)


what is used :Ö Centroid tracking + pinhole back-projection: also called Sparse 2.5D Tracking
 pixel into an $(X, Y, Z)$ coordinate, the script uses the Pinhole Camera Model.

 now only include z axis
 so 1st boundin box, and cnetorid then do vx vy vz for that centroid a

 now using the built in flagg, which check for object closing up and distance changing, and seeing if my code runs properly


 risk=round(0.7×risk_ttc+0.3×risk_speed)

 TEST_MODE = True  # True: synthetic circle (no camera) | False: live RealSense camera at run_risk_roi_red_black
 change here for true - flase flag


 Test audio only:
python realsense_camera/python/launch/run_risk_roi_red_black.py --audio-test
Run pipeline without sound:
python realsense_camera/python/launch/run_risk_roi_red_black.py --no-audio

    changed blobdetector to HSV again 

