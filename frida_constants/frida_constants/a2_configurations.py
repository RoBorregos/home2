# Named poses for the Agility A2 right arm (openarm_right_joint1..7), same names as the xArm ones.
# Stare poses were solved by IK so the wrist ZED looks at the same table area the xArm stares at.


def _pose(*deg):
    return {
        "joints": {f"openarm_right_joint{i}": v for i, v in enumerate(deg, start=1)},
        "degrees": True,
    }


# ZED at (0.25, -0.15, 1.2) in base_link looking at the dining table center
TABLE_STARE = _pose(35.4, 154.0, 11.8, 105.4, -47.3, 5.2, 39.7)
# ZED at (0.3, -0.3, 1.3) looking straight ahead
FRONT_STARE = _pose(27.4, 153.3, -21.1, 88.7, -43.5, -3.8, -11.8)
# ZED at (0.1, -0.35, 1.2) looking at the side table on the robot's right
SIDE_TABLE_STARE = _pose(45.3, 174.4, -18.5, 84.0, -70.2, -31.8, 69.9)
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
