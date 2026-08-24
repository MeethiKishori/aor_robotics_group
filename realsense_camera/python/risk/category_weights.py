LIVING  = "LIVING"   # humans, animals
DYNAMIC = "DYNAMIC"  # other robots, vehicles
STATIC  = "STATIC"   # walls, furniture, other stationary obstacles

CATEGORY_WEIGHTS = {
    LIVING:  3.0,
    DYNAMIC: 2.0,
    STATIC:  1.0,
}

# Maps a detector label (e.g. from YoloDetector) to one of the three hazard groups above.
LABEL_TO_CATEGORY = {
    "HUMAN": LIVING,
    "ROBOT": DYNAMIC,
}


def category_weight(label):
    """w(c_i): hazard weight for a detection label. Unknown labels default to STATIC."""
    category = LABEL_TO_CATEGORY.get(label, STATIC)
    return CATEGORY_WEIGHTS[category]
