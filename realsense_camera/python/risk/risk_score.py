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


def select_strongest_detections(detections, max_count=3, allowed_labels=("RED", "BLACK")):
    # Pick top-N strongest detections using a simple closeness+size score.
    # Higher area and lower distance both increase strength.
    scored = []
    for d in detections:
        if d.get("label") not in allowed_labels:
            continue

        dist = float(d.get("distance_m", 0.0))
        area = float(d.get("area", 0.0))
        if dist <= 0 or area <= 0:
            continue

        # Close objects and larger blobs get priority.
        strength = area / max(dist, 0.10)
        scored.append((strength, d))

    scored.sort(key=lambda x: x[0], reverse=True)
    return [item[1] for item in scored[:max_count]]


def compute_ttc_risk(distance_m, speed_mps):
    # Risk model blending TTC + speed + distance (0-10).
    if distance_m == float("inf"):
        return 0, float("inf")

    # TTC = time to collision.
    ttc = distance_m / max(speed_mps, 0.05)

    # TTC contribution (60% weight).
    if ttc < 2.0:
        risk_ttc = 10
    elif ttc < 4.0:
        risk_ttc = 8
    elif ttc < 6.0:
        risk_ttc = 5
    elif ttc < 8.0:
        risk_ttc = 2
    else:
        risk_ttc = 0

    # Speed contribution (20% weight).
    if speed_mps < 0.15:
        risk_speed = 0      # near-static, likely noise
    elif speed_mps < 0.4:
        risk_speed = 2      # slow drift / creeping
    elif speed_mps < 0.8:
        risk_speed = 5      # moderate approach (slow walk)
    elif speed_mps < 1.5:
        risk_speed = 7      # fast approach (brisk walk)
    else:
        risk_speed = 10     # very fast (running / vehicle)

    # Distance contribution (20% weight) — raises alarm even for slow/static close objects.
    if distance_m < 0.20:
        risk_dist = 10      # under 20 cm: critical
    elif distance_m < 0.40:
        risk_dist = 8       # 20–40 cm: very close
    elif distance_m < 1.0:
        risk_dist = 5       # 40 cm – 1.0 m: moderate
    elif distance_m < 2.00:
        risk_dist = 2       # 1.0 – 2 m: low
    else:
        risk_dist = 0       # over 2 m: safe

    risk = round(0.6 * risk_ttc + 0.2 * risk_speed + 0.2 * risk_dist)
    risk = int(max(0, min(10, risk)))

    # Pure distance score — runs in parallel, not blended.
    # Whichever is more dangerous (formula vs distance alone) wins.
    if distance_m < 0.40:
        dist_only = 9
    elif distance_m < 0.70:
        dist_only = 6
    elif distance_m < 1.20:
        dist_only = 3
    else:
        dist_only = 0

    risk = max(risk, dist_only)

    return int(risk), float(ttc)


def risk_label(risk):
    # Convert numeric risk (0-10) to a human-readable label and a BGR draw color.
    # BGR = Blue, Green, Red (OpenCV's channel order, not RGB).
    if risk >= 7:
        return "DANGER",   (0, 0, 255)     # red box on screen
    if risk >= 4:
        return "MODERATE", (0, 255, 255)   # yellow box on screen
    return "LOW",          (0, 255, 0)     # green box on screen
