from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class HazardObject:
    """One detected object's category and spatial distance (c_i, d_i in the risk model)."""
    category: str                       # e.g. "HUMAN", "ROBOT"
    distance_m: float                   # d_i, closest distance to the robot
    box: Tuple[int, int, int, int]      # (x, y, w, h)
    confidence: float = 0.0


class HazardRepresentation:
    """Holds the current frame's hazard objects and their distances.

    This is the perception output handed to the risk model
    (R_i = w(c_i) * g(d_i) * (1 + alpha * v_tilde)).
    """

    def __init__(self):
        self.objects: List[HazardObject] = []

    def update(self, detections):
        """Rebuild the hazard list from detection dicts that already have distance_m set."""
        self.objects = [
            HazardObject(
                category=d["label"],
                distance_m=float(d.get("distance_m", 0.0)),
                box=d["box"],
                confidence=float(d.get("confidence", 0.0)),
            )
            for d in detections
        ]
        return self.objects

    def distances(self, category: Optional[str] = None) -> List[float]:
        """Distance list for future use (e.g. nearest-object or risk aggregation)."""
        return [o.distance_m for o in self.objects if category is None or o.category == category]

    def nearest(self, category: Optional[str] = None) -> Optional[HazardObject]:
        candidates = [o for o in self.objects if o.distance_m > 0 and (category is None or o.category == category)]
        if not candidates:
            return None
        return min(candidates, key=lambda o: o.distance_m)
