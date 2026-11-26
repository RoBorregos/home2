"""Shelf level numbering shared by the task managers. No ROS imports."""

# Boards below this base_link height are too low for the arm: announce-only
MIN_REACHABLE_SHELF_Z = 0.20


def levels_from_sorted_heights(heights):
    """Map measured surface heights to the {1,2,3} level dict (1 = lowest)."""
    ordered = sorted(float(h) for h in heights)
    return {i + 1: h for i, h in enumerate(ordered)}
