from .category_weights import category_weight
from .distance_risk import distance_risk_g

ALPHA = 0.5  # default influence of normalized robot velocity on risk


def object_risk(label, distance_m, velocity_norm=0.0, alpha=ALPHA):
    """R_i = w(c_i) * g(d_i) * (1 + alpha * v_tilde)."""
    w = category_weight(label)
    g = distance_risk_g(distance_m)
    return w * g * (1.0 + alpha * velocity_norm)


def compute_object_risks(hazard_objects, velocity_norm=0.0, alpha=ALPHA):
    """Per-object (HazardObject, R_i) pairs for the current frame."""
    return [
        (o, object_risk(o.category, o.distance_m, velocity_norm, alpha))
        for o in hazard_objects
    ]


def environmental_risk(hazard_objects, velocity_norm=0.0, alpha=ALPHA):
    """R_env = max_i R_i over a list of HazardObject (or anything with .category/.distance_m)."""
    risks = compute_object_risks(hazard_objects, velocity_norm, alpha)
    if not risks:
        return 0.0
    return max(r for _, r in risks)
