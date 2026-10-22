# Named poses for the Agility A2 right arm (openarm_right_joint1..7), same names as the xArm ones.
# Stare poses were solved by IK so the wrist ZED looks at the same table area the xArm stares at.


def _pose(*deg):
    return {
        "joints": {f"openarm_right_joint{i}": v for i, v in enumerate(deg, start=1)},
        "degrees": True,
    }


# ZED at (0.18, -0.15, 1.2) in base_link looking at the dining table objects, image upright
TABLE_STARE = _pose(43.9, 150.6, 20.1, 101.3, -54.3, -13.5, 53.9)
# ZED at (0.3, -0.3, 1.3) looking straight ahead
FRONT_STARE = _pose(27.4, 153.3, -21.1, 88.7, -43.5, -3.8, -11.8)
# ZED at (0.0, -0.35, 1.15) looking at the side table on the robot's right, image upright
SIDE_TABLE_STARE = _pose(77.9, 177.8, -18.9, 98.0, -77.1, -32.2, 76.1)
# Forearm raised forward, clear of the base and the table (the sim spawn pose)
HOME = _pose(0.0, 17.2, 0.0, 126.1, 0.0, 0.0, 0.0)

A2_CONFIGURATIONS = {
    "table_stare": TABLE_STARE,
    "front_stare": FRONT_STARE,
    "side_table_stare": SIDE_TABLE_STARE,
    "flat_stare": TABLE_STARE,
    "look_side_stare": TABLE_STARE,
    "pick_stare_at_table": TABLE_STARE,
    "nav_pose": HOME,
    "carry_pose": HOME,
    "home": HOME,
}
