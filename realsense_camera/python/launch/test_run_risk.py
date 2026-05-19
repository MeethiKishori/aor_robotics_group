import os
import sys
import cv2
import numpy as np
import time

# Allow imports from sibling modules.
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(THIS_DIR)
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from actuators.tower import SignalTowerController
from risk.risk_score import compute_ttc_risk, risk_label

# --- WINDOW CONFIGURATION ---
WIDTH, HEIGHT = 640, 480
FPS = 30

# --- SIMULATION STATE ---
current_dist = 5.5  # Start far away (meters)
speed = -0.5        # Negative means moving TOWARD the camera
dt = 0.05           # Time step fraction

def set_tower_state(tower, risk_text):
    if risk_text == "LOW":
        tower.green()
        #tower.buzzer(False)
        #tower.flash(1)
    elif risk_text == "MODERATE":
        tower.yellow()
        #tower.buzzer(False)
       # tower.flash(3)
    elif risk_text == "DANGER":
        tower.red()
        #tower.buzzer(True)
       # tower.flash(3)

print("Running window test. Press 'q' inside the window to quit.")

tower = None
try:
    tower = SignalTowerController()
except Exception as e:
    print(f"Warning: cannot initialize tower controller: {e}")
    print("Proceeding without hardware output.")

try:
    while True:
        # 1. Calculate the next distance step
        current_dist = current_dist + (speed * dt)

        # 2. Reversal logic to keep it bouncing between 0.5m and 5.5m
        if current_dist <= 0.5:
            current_dist = 0.5
            speed = abs(speed)   # Move away
        elif current_dist >= 5.5:
            current_dist = 5.5
            speed = -abs(speed)  # Move closer

        # 3. Translate distance to a visual circle radius (Scale fixed for 5.5m max)
        # 0.5m -> radius 150px | 5.5m -> radius 20px
        radius = int(150 - (current_dist - 0.5) * 26)
        radius = max(10, radius) # Safety cap

        # 4. Create a blank gray image window canvas
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 220

        # 5. Draw a 90% Center Region of Interest (ROI) box
        # 90% active means a 5% margin on each outer side
        rx1, ry1 = int(WIDTH * 0.05), int(HEIGHT * 0.05)
        rx2, ry2 = int(WIDTH * 0.95), int(HEIGHT * 0.95)
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 0, 255), 2) # Red Box

        # 6. Draw the dark gray moving circle right in the center
        center_x, center_y = WIDTH // 2, HEIGHT // 2
        cv2.circle(frame, (center_x, center_y), radius, (40, 40, 40), -1)

        # 7. Compute risk using the shared risk scoring model.
        risk, ttc = compute_ttc_risk(current_dist, abs(speed))
        risk_text, text_color = risk_label(risk)

        # 8. Draw HUD Data Labels onto the window frame
        cv2.putText(frame, f"RISK: {risk_text} ({risk})", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
        cv2.putText(frame, f"Distance: {current_dist:.2f} m", (30, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        cv2.putText(frame, f"Speed: {abs(speed):+.2f} m/s", (30, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        cv2.putText(frame, f"TTC: {ttc:.2f} s", (30, 150), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        # 9. Send tower state and refresh window frame display.
        if tower:
            set_tower_state(tower, risk_text)

        cv2.imshow("Risk Simulation Window", frame)

        # Control loop frame rate (wait roughly 50ms)
        if cv2.waitKey(int(dt * 1000)) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nSimulation stopped.")

finally:
    if tower:
        tower.close()
    cv2.destroyAllWindows()