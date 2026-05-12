def risk_from_distance(distance_m):
    # Convert nearest detected object distance (metres) to a risk score 0-10.
    # Closer = higher risk. These thresholds are easy to tune.
    if distance_m <= 0:
        return 0       # 0 means no valid depth reading, treat as safe

    if distance_m < 0.40:
        return 10      # under 40 cm -> critical, very close
    if distance_m < 0.70:
        return 8       # 40-70 cm -> very dangerous
    if distance_m < 1.20:
        return 6       # 70 cm - 1.2 m -> moderate risk
    if distance_m < 2.00:
        return 3       # 1.2 - 2 m -> low risk
    return 1           # over 2 m -> almost safe (1 not 0 because object is still visible)


def nearest_distance_from_detections(detections, allowed_labels=("RED", "BLACK")):
    # Get nearest valid distance from selected object labels.
    # Returns inf if nothing valid is found.
    nearest = float("inf")
    for d in detections:
        if d.get("label") not in allowed_labels:
            continue
        dist = float(d.get("distance_m", 0.0))
        if dist > 0:
            nearest = min(nearest, dist)
    return nearest


def compute_ttc_risk(distance_m, speed_mps):
    # Reusable TTC+speed risk model (0-10). Safer for sharing across projects.
    if distance_m == float("inf"):
        return 0, float("inf")

    # TTC = time to collision.
    ttc = distance_m / max(speed_mps, 0.05)

    # TTC contribution.
    if ttc < 0.5:
        risk_ttc = 10
    elif ttc < 1.0:
        risk_ttc = 8
    elif ttc < 2.0:
        risk_ttc = 5
    elif ttc < 4.0:
        risk_ttc = 2
    else:
        risk_ttc = 0

    # Speed contribution.
    if speed_mps < 0.2:
        risk_speed = 1
    elif speed_mps < 0.5:
        risk_speed = 3
    elif speed_mps < 1.0:
        risk_speed = 6
    else:
        risk_speed = 8

    risk = round(0.7 * risk_ttc + 0.3 * risk_speed)
    return int(max(0, min(10, risk))), float(ttc)


def risk_label(risk):
    # Convert numeric risk (0-10) to a human-readable label and a BGR draw color.
    # BGR = Blue, Green, Red (OpenCV's channel order, not RGB).
    if risk >= 7:
        return "DANGER",   (0, 0, 255)     # red box on screen
    if risk >= 4:
        return "MODERATE", (0, 255, 255)   # yellow box on screen
    return "LOW",          (0, 255, 0)     # green box on screen
