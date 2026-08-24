RMAX  = 30.0   # bound applied once an object is effectively touching the robot
D_MIN = 0.10   # metres; distances at or below this are clamped to RMAX


def distance_risk_g(distance_m):
    """g(d_i): inverse-distance risk function, bounded for numerical stability."""
    if distance_m <= D_MIN:
        return RMAX
    return 1.0 / distance_m
