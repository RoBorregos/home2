FRONT_STARE = {
    "joints": {
        "joint1": -90.0,
        "joint2": -45.0,
        "joint3": -90.0,
        "joint4": 0.0,
        "joint5": 0.0,
        "joint6": 45.0,
    },
    "degrees": True,
}

FRONT_LOW_STARE = {
    "joints": {
        "joint1": -90.0,
        "joint2": -45.0,
        "joint3": -90.0,
        "joint4": 0.0,
        "joint5": 20.0,
        "joint6": 45.0,
    },
    "degrees": True,
}

TABLE_STARE = {
    "joints": {
        "joint1": -90.0,
        "joint2": -80.0,
        "joint3": -70.0,
        "joint4": 0.0,
        "joint5": 60.0,
        "joint6": 45.0,
    },
    "degrees": True,
}

RECEIVE_OBJECT = {
    "joints": {
        "joint1": -1.6047106981277466,
        "joint2": -1.1695280075073242,
        "joint3": -0.5476177334785461,
        "joint4": 0.013918958604335785,
        "joint5": 0.04733673110604286,
        "joint6": 0.7352485060691833,
    },
    "degrees": False,
}

PICK_STARE_AT_TABLE = {
    "joints": {
        "joint1": -90.0,
        "joint2": -10.0,
        "joint3": -170.0,
        "joint4": 0.0,
        "joint5": 100.0,
        "joint6": 45.0,
    },
    "degrees": True,
}

# Creditos a dominguez


NAV_POSE = {
    "joints": {
        "joint1": -90.0,
        "joint2": -60.0,
        "joint3": -55.0,
        "joint4": 0,
        "joint5": 25.0,
        "joint6": 45.0,
    },
    "degrees": True,
}


XARM_CONFIGURATIONS = {
    "front_stare": FRONT_STARE,
    "front_low_stare": FRONT_LOW_STARE,
    "table_stare": TABLE_STARE,
    "receive_object": RECEIVE_OBJECT,
    "pick_stare_at_table": PICK_STARE_AT_TABLE,
    "nav_pose": NAV_POSE,
}
