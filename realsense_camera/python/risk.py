import time
import numpy as np
import pyrealsense2 as rs

# ANSI colors for terminal output
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
RESET  = "\033[0m"


def risk_label(risk):
    # Divide 0-10 risk into three human-readable levels with colors.
    if risk >= 7:
        return RED, "DANGER"
    elif risk >= 4:
        return YELLOW, "MODERATE"
    else:
        return GREEN, "LOW"


def compute_risk(d_front, speed_est):
    # TTC = Time To Collision in seconds.
    # Divide distance by speed (minimum 0.05 to avoid division by zero).
    ttc = d_front / max(speed_est, 0.05)

    # Map TTC to a risk score 0-10.
    if ttc < 0.5:    risk_ttc = 10   # less than 0.5 s -> critical
    elif ttc < 1.0:  risk_ttc = 8    # 0.5-1 s -> very dangerous
    elif ttc < 2.0:  risk_ttc = 5    # 1-2 s -> moderate
    elif ttc < 4.0:  risk_ttc = 2    # 2-4 s -> low
    else:            risk_ttc = 0    # > 4 s -> safe

    # Map speed to a risk score 0-8.
    if speed_est < 0.2:    risk_speed = 1   # almost stationary
    elif speed_est < 0.5:  risk_speed = 3   # slow
    elif speed_est < 1.0:  risk_speed = 6   # moderate speed
    else:                  risk_speed = 8   # fast

    # Final risk: 70% from TTC, 30% from speed. Clamped to 0-10.
    risk = round(0.7 * risk_ttc + 0.3 * risk_speed)
    return int(max(0, min(10, risk))), ttc


def main():
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable depth stream: 640x480, 16-bit format, 30 FPS.
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # Enable IMU streams. D435i: accel @ 250 Hz, gyro @ 200 Hz.
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    config.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)

    profile = pipeline.start(config)

    # depth_scale converts raw 16-bit pixel values to metres.
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    speed_est   = 0.0          # camera motion speed in m/s (from IMU accel)
    accel_mag   = 0.0          # total IMU acceleration magnitude in m/s²
    d_front_prev = float("inf") # d_front from the previous loop (to compute closure rate)
    last_t      = time.time()

    print("Risk monitor running... Press Ctrl+C to stop.\n")
    try:
        while True:
            frames = pipeline.wait_for_frames()   # block until next frameset arrives

            depth = frames.get_depth_frame()
            if not depth:
                continue   # skip if no depth frame yet

            # Read accelerometer from IMU frames.
            for f in frames:
                if f.is_motion_frame() and f.get_profile().stream_type() == rs.stream.accel:
                    m = f.as_motion_frame().get_motion_data()
                    # Magnitude of accel vector: sqrt(x²+y²+z²). Includes gravity (~9.81).
                    accel_mag = float(np.linalg.norm([m.x, m.y, m.z]))

            # Time delta since last loop iteration.
            now  = time.time()
            dt   = max(now - last_t, 1e-3)
            last_t = now

            # Remove gravity from accel magnitude to get motion-only acceleration.
            # Integrate over time to get speed estimate: v += a * dt
            lin_accel  = max(0.0, accel_mag - 9.81)
            speed_est += lin_accel * dt
            speed_est *= 0.98                        # decay to prevent drift
            speed_est  = max(0.0, min(5.0, speed_est))

            # Convert depth frame to metres using depth_scale.
            depth_m = np.asanyarray(depth.get_data()) * depth_scale

            # Get nearest obstacle in center 30% of image.
            h, w  = depth_m.shape
            roi   = depth_m[int(0.35*h):int(0.65*h), int(0.35*w):int(0.65*w)]
            valid = roi[roi > 0]
            d_front = float(np.percentile(valid, 10)) if valid.size > 0 else float("inf")

            # Closure rate = how fast d_front shrinks per second.
            # This is the RELATIVE velocity between camera and obstacle —
            # it automatically accounts for both moving toward each other.
            # Example: camera at 0.5 m/s + obstacle at 0.5 m/s = closure 1.0 m/s
            closure_rate = max(0.0, (d_front_prev - d_front) / dt)
            d_front_prev = d_front   # save for next loop

            # Use closure_rate directly as effective speed for TTC.
            # cam speed is shown separately for information only.
            risk, ttc = compute_risk(d_front, closure_rate)
            color, label = risk_label(risk)

            print(
                f"{color}[{label}] risk={risk}/10  "
                f"dist={d_front:.2f}m  cam={speed_est:.2f}m/s  closure={closure_rate:.2f}m/s  ttc={ttc:.2f}s{RESET}    \r",
                end="", flush=True,
            )

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()



def clamp(value, lo, hi):
	# Restrict 'value' to stay between lo and hi.
	# Example: clamp(12, 0, 10) -> 10  |  clamp(-1, 0, 10) -> 0
	return max(lo, min(hi, value))


def risk_from_ttc(ttc_s):
	# TTC = Time To Collision (seconds). Lower TTC = closer to hitting something = higher risk.
	if ttc_s < 0.5:   # less than half a second until collision
		return 10      # critical
	if ttc_s < 1.0:   # less than 1 second
		return 8       # very dangerous
	if ttc_s < 2.0:   # 1-2 seconds away
		return 5       # moderate risk
	if ttc_s < 4.0:   # 2-4 seconds away
		return 2       # low risk
	return 0           # more than 4 seconds, safe


def risk_from_speed(v_mps):
	# Map forward speed (meters per second) to a risk contribution.
	# Fast movement adds risk even if no obstacle is nearby yet.
	if v_mps < 0.2:   # very slow or stationary
		return 1
	if v_mps < 0.5:   # walking speed
		return 3
	if v_mps < 1.0:   # jogging speed
		return 6
	return 8           # fast (above 1 m/s)


def front_distance_m(depth_m):
	# Find the nearest obstacle in the center 30% x 30% region of the depth image.
	h, w = depth_m.shape                       # image height and width in pixels
	x0, x1 = int(0.35 * w), int(0.65 * w)     # horizontal bounds of center ROI
	y0, y1 = int(0.35 * h), int(0.65 * h)     # vertical bounds of center ROI
	roi = depth_m[y0:y1, x0:x1]               # crop to center region only
	valid = roi[roi > 0]                       # keep only pixels with real depth (0 = invalid)
	if valid.size == 0:                        # no valid depth in ROI (out of range)
		return float("inf")                    # treat as infinitely far away
	# Use 10th percentile instead of min to ignore noisy/outlier pixels.
	# Means: 90% of pixels in the ROI are farther than this value.
	return float(np.percentile(valid, 10))


def main():
	pipeline = rs.pipeline()   # create the RealSense data streaming object
	use_imu = True             # assume IMU is available until proven otherwise

	# Build a stream config requesting depth + accelerometer + gyroscope.
	config = rs.config()
	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
	# D435i: accel supports 250 Hz or 63 Hz. Gyro supports 400 Hz or 200 Hz.
	config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
	config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
	try:
		profile = pipeline.start(config)   # start streaming all three sensors
	except RuntimeError as e:
		# If the camera doesn't support the requested streams, fall back to depth only.
		if "Couldn't resolve requests" not in str(e):
			raise   # re-raise any other unexpected errors
		use_imu = False
		config = rs.config()
		config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		profile = pipeline.start(config)   # start depth-only streaming
		print("IMU streams unavailable on this device/config. Running depth-only risk mode.")
	else:
		print("IMU streams enabled (depth + accel + gyro).")

	# depth_scale converts raw 16-bit integer depth values to metres.
	# Example: raw value 1000 * depth_scale = distance in metres.
	depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

	speed_est = 0.0    # estimated forward speed in m/s (starts at 0)
	accel_mag = 0.0    # magnitude of acceleration vector from IMU in m/s²
	gyro_mag  = 0.0    # magnitude of angular velocity vector from gyro in rad/s
	last_t = time.time()   # timestamp of previous loop iteration

	print("Streaming risk... Press Ctrl+C to stop")
	try:
		while True:
			# Block here until a complete set of frames arrives from the camera.
			frames = pipeline.wait_for_frames()

			depth = frames.get_depth_frame()   # extract depth frame from the frameset
			if not depth:
				continue   # skip this iteration if depth frame is missing

			if use_imu:
				# Loop over all frames in the frameset and look for motion frames.
				for f in frames:
					if not f.is_motion_frame():
						continue   # skip non-IMU frames (depth, color, etc.)
					motion = f.as_motion_frame().get_motion_data()   # get x, y, z values
					if f.get_profile().stream_type() == rs.stream.accel:
						# np.linalg.norm computes vector magnitude: sqrt(x² + y² + z²)
						# Gives total acceleration in m/s² regardless of direction.
						accel_mag = float(np.linalg.norm([motion.x, motion.y, motion.z]))
					elif f.get_profile().stream_type() == rs.stream.gyro:
						# Same for gyro: total rotation speed in rad/s.
						gyro_mag = float(np.linalg.norm([motion.x, motion.y, motion.z]))

			now = time.time()
			dt = max(now - last_t, 1e-3)   # time since last loop; min 1ms to avoid divide-by-zero
			last_t = now                    # save timestamp for next loop

			if use_imu:
				# Subtract gravity (9.81 m/s²) to isolate motion-caused acceleration.
				# max(..., 0) prevents negative values when camera is still.
				lin_accel = max(0.0, accel_mag - 9.81)
				# Integrate: v += a * dt  (speed = speed + acceleration × time)
				speed_est += lin_accel * dt
				# Decay factor: speed loses 2% per loop to prevent drift accumulation.
				speed_est *= 0.98
				# Clamp speed to a physically reasonable range [0, 5 m/s].
				speed_est = clamp(speed_est, 0.0, 5.0)
			else:
				speed_est = 0.0   # no IMU: assume stationary, risk comes from TTC only

			# Convert raw 16-bit depth frame to a 2D NumPy array.
			depth_u16 = np.asanyarray(depth.get_data())
			# Multiply by depth_scale to convert each pixel's value to metres.
			depth_m = depth_u16 * depth_scale
			# Get nearest obstacle distance in the center region of the image.
			d_front = front_distance_m(depth_m)

			# TTC = distance / speed. Use 0.05 as minimum speed to avoid division by zero.
			ttc = d_front / max(speed_est, 0.05)
			risk_ttc   = risk_from_ttc(ttc)           # 0-10 score based on time to collision
			risk_speed = risk_from_speed(speed_est)   # 0-8 score based on speed alone

			stability_penalty = 0   # extra risk added when camera is rotating fast
			if use_imu:
				if gyro_mag > 3.0:    # very fast rotation (sharp turn or unstable)
					stability_penalty = 2
				elif gyro_mag > 1.5:  # moderate rotation
					stability_penalty = 1

			# Final risk formula: 70% TTC risk + 30% speed risk + stability penalty.
			risk = int(round(0.7 * risk_ttc + 0.3 * risk_speed + stability_penalty))
			# Clamp final value to valid range 0-10.
			risk = int(clamp(risk, 0, 10))

			# Print one updating line. \r moves cursor to start of line so the next
			# print overwrites it (no scrolling output).
			print(
				f"d_front={d_front:5.2f}m  v={speed_est:4.2f}m/s  ttc={ttc:5.2f}s  "
				f"gyro={gyro_mag:4.2f}rad/s  risk={risk:2d}/10\r",
				end="",      # don't add a newline
				flush=True,  # force output to appear immediately (not buffered)
			)
	except KeyboardInterrupt:
		print("\nStopped")   # print on a fresh line after the \r output
	finally:
		pipeline.stop()   # always release the camera, even if an error occurred


if __name__ == "__main__":
	main()   # only run main() when this file is executed directly, not when imported
