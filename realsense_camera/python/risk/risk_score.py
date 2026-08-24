def distance_risk(distance_m):
    # Convert nearest detected object distance (metres) to a risk score 0-10.
    # Closer = higher risk. These thresholds are easy to tune.
    if distance_m <= 0:
        return 0       # no valid depth reading, treat as safe

    if distance_m < 0.40:
        return 10      # under 40 cm -> critical, very close
    if distance_m < 0.70:
        return 8       # 40-70 cm -> very dangerous
    if distance_m < 1.20:
        return 6       # 70 cm - 1.2 m -> moderate risk
    if distance_m < 2.00:
        return 3       # 1.2 - 2 m -> low risk
    return 1           # over 2 m -> almost safe (1 not 0 because object is still visible)


def risk_from_distance(distance_m):
    return distance_risk(distance_m)


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


def select_strongest_detections(detections, max_count=3, allowed_labels=("RED", "BLACK")):
    # Pick top-N strongest detections using a simple closeness+size score.
    # Higher area and lower distance both increase strength.
    scored = []
    for d in detections:
        if d.get("label") not in allowed_labels:
            continue

        dist = float(d.get("distance_m", 0.0))
        area = float(d.get("area", 0.0))
        if area <= 0:
            continue

        # Unknown depth (0) treated as a close unknown — use a conservative 0.5 m proxy
        # so the object still participates in risk ranking rather than being dropped.
        effective_dist = dist if dist > 0 else 0.5
        strength = area / max(effective_dist, 0.10)
        scored.append((strength, d))

    scored.sort(key=lambda x: x[0], reverse=True)
    return [item[1] for item in scored[:max_count]]


def compute_ttc_risk(distance_m, speed_mps=0.0):
    if distance_m == float("inf"):
        return 0, float("inf")

    # speed=0 → stationary, TTC=inf, risk from distance only.
    if speed_mps <= 0:
        return int(distance_risk(distance_m)), float("inf")

    ttc  = distance_m / speed_mps
    risk = distance_risk(distance_m)
    if ttc < 2.0:
        risk = min(10, risk + 4)
    elif ttc < 4.0:
        risk = min(10, risk + 2)
    return int(risk), float(ttc)


def risk_label(risk):
    # Convert numeric risk (0-10) to a human-readable label and a BGR draw color.
    # BGR = Blue, Green, Red (OpenCV's channel order, not RGB).
    if risk >= 7:
        return "DANGER",   (0, 0, 255)     # red box on screen
    if risk >= 4:
        return "MODERATE", (0, 255, 255)   # yellow box on screen
    return "LOW",          (0, 255, 0)     # green box on screen
