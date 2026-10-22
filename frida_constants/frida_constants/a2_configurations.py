# Named poses for the Agility A2 right arm (openarm_right_joint1..7), same names as the xArm ones.
# The ZED is a fixed head camera, so the stare poses only move the arm out of its view.


def _pose(*deg):
    return {
        "joints": {f"openarm_right_joint{i}": v for i, v in enumerate(deg, start=1)},
        "degrees": True,
    }


# Right arm stretched out to the side at shoulder height, clear of the head camera view
TABLE_STARE = _pose(0.0, 90.0, 0.0, 17.2, 0.0, 0.0, 0.0)
SIDE_TABLE_STARE = TABLE_STARE
# Forearm raised forward, clear of the base and the table (the sim spawn pose)
HOME = _pose(0.0, 17.2, 0.0, 126.1, 0.0, 0.0, 0.0)

A2_CONFIGURATIONS = {
    "table_stare": TABLE_STARE,
    "front_stare": TABLE_STARE,
    "side_table_stare": SIDE_TABLE_STARE,
    "flat_stare": TABLE_STARE,
    "look_side_stare": TABLE_STARE,
    "pick_stare_at_table": TABLE_STARE,
    "nav_pose": HOME,
    "carry_pose": HOME,
    "home": HOME,
}
