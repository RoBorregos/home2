#!/usr/bin/env python3
"""Headless ZED hand-eye calibration (eye-in-hand, ChArUco board)

  ros2 run perception_3d hand_eye_calibration.py capture [--ros-args -p out_dir:=DIR -p square_len:=0.0301]
      Hand-guide the arm (/xarm/set_mode {data: 2}, /xarm/set_state {data: 0}), hold still,
      press Enter to take a sample, q + Enter to finish. The node never commands the arm.
  ros2 run perception_3d hand_eye_calibration.py solve DIR
      Prints the <origin .../> line for the ZED joint of gripper.xacro. Needs only numpy,
      scipy and cv2

Use the same printed board for every calibration. square_len is only for
a printed square that measures differently from SQUARE_LEN; then edit SQUARE_LEN once.
"""

import itertools
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# Board geometry (m). The printable board is config/charuco_7x5_30mm_DICT4X4_50_A4.pdf, rendered
# from these constants: print it at 100%, check 210 x 150 mm with calipers, keep that one board.
SQUARES_X, SQUARES_Y = 7, 5
SQUARE_LEN, MARKER_LEN = 0.030, 0.022
DICT_NAME = "DICT_4X4_50"
GRIPPER, ZED_LINK = "gripper", "zed_camera_link"
METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def mat(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def inv(T):
    return mat(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def angle(Ra, Rb):
    """Degrees between two rotation matrices."""
    return np.degrees(Rotation.from_matrix(Ra.T @ Rb).magnitude())


def solve(out_dir):
    data = json.loads((Path(out_dir) / "samples.json").read_text())
    A = [np.array(s["T_base_eef"]) for s in data["samples"]]
    B = [np.array(s["T_cam_target"]) for s in data["samples"]]
    if len(A) < 10:
        sys.exit(f"{len(A)} samples, need at least 10 (20+ recommended)")
    if len(A) < 15:
        print(f"WARN: only {len(A)} samples, 20+ recommended")

    # calibrateHandEye returns X = T_eef_optical. T_base_target = A X B must be the same
    # for every sample, so its spread is the quality number.
    res = {}
    for name, method in METHODS.items():
        R, t = cv2.calibrateHandEye(
            [a[:3, :3] for a in A],
            [a[:3, 3] for a in A],
            [b[:3, :3] for b in B],
            [b[:3, 3] for b in B],
            method=method,
        )
        X = mat(R, t.ravel())
        P = [a @ X @ b for a, b in zip(A, B)]
        pos = np.array([p[:3, 3] for p in P])
        rots = Rotation.from_matrix([p[:3, :3] for p in P])
        mm = 1000 * np.sqrt(((pos - pos.mean(0)) ** 2).sum(1).mean())
        deg = np.degrees(np.sqrt(((rots.mean().inv() * rots).magnitude() ** 2).mean()))
        res[name] = (X, mm, deg)
    best = min(res, key=lambda k: res[k][1])
    X, mm, deg = res[best]

    print(f"{len(A)} samples, board {data['board']}")
    for k, (_, m, d) in res.items():
        print(
            f"  {k:11s} spread {m:5.2f} mm {d:5.2f} deg{'   <- best' if k == best else ''}"
        )
    gap_mm = max(
        1000 * np.linalg.norm(a[0][:3, 3] - b[0][:3, 3])
        for a, b in itertools.combinations(res.values(), 2)
    )
    gap_deg = max(
        angle(a[0][:3, :3], b[0][:3, :3])
        for a, b in itertools.combinations(res.values(), 2)
    )
    print(f"  methods disagree by up to {gap_mm:.2f} mm, {gap_deg:.2f} deg")

    # eef -> gripper -> zed_camera_link -> optical = X, so the xacro joint is:
    F = data["frames"]
    T = inv(np.array(F["T_eef_gripper"])) @ X @ inv(np.array(F["T_zedlink_optical"]))
    cur = np.array(F["T_gripper_zedlink"])
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")  # URDF rpy = extrinsic xyz
    ok = mm <= 3 and deg <= 0.5 and gap_mm <= 3 and gap_deg <= 0.5
    print(
        f"{'PASS' if ok else 'WARN: more rotation diversity, or check the board size and the mount'}"
    )
    print(
        f'<origin xyz="{T[0, 3]:.4f} {T[1, 3]:.4f} {T[2, 3]:.4f}" rpy="{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}" />'
    )
    d = 1000 * (T[:3, 3] - cur[:3, 3])
    print(
        f"delta vs current: {d[0]:.1f} {d[1]:.1f} {d[2]:.1f} mm, {angle(cur[:3, :3], T[:3, :3]):.2f} deg"
    )
    return {"T": T, "rpy": rpy, "method": best, "spread_mm": mm, "spread_deg": deg}


def capture():
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node("hand_eye_calibration")
    p = {
        k: node.declare_parameter(k, v).value
        for k, v in {
            "image_topic": "/zed/zed_node/rgb/color/rect/image",
            "info_topic": "/zed/zed_node/rgb/color/rect/camera_info",
            "base_frame": "link_base",
            "eef_frame": "link_eef",
            "min_corners": 15,
            "max_reproj_px": 0.8,
            "square_len": SQUARE_LEN,
            "out_dir": time.strftime("hand_eye_%Y%m%d_%H%M%S"),
        }.items()
    }
    marker_len = MARKER_LEN * p["square_len"] / SQUARE_LEN
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        p["square_len"],
        marker_len,
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME)),
    )
    detector = cv2.aruco.CharucoDetector(board)
    bridge = CvBridge()
    tf = Buffer()
    TransformListener(tf, node)

    last = {
        "info": None,
        "frame": None,
    }

    def on_image(msg):
        if last["info"]:
            last["frame"] = (msg, last["info"])

    node.create_subscription(
        CameraInfo,
        p["info_topic"],
        lambda m: last.update(info=m),
        qos_profile_sensor_data,
    )
    node.create_subscription(Image, p["image_topic"], on_image, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def lookup(parent, child, stamp=None):  # stamp None = latest
        stamp = Time() if stamp is None else stamp
        t = tf.lookup_transform(parent, child, stamp, Duration(seconds=2.0)).transform
        q, v = t.rotation, t.translation
        return mat(
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [v.x, v.y, v.z]
        )

    out = Path(p["out_dir"])
    out.mkdir(parents=True, exist_ok=True)
    f = out / "samples.json"
    data = (
        json.loads(f.read_text())
        if f.exists()
        else {  # resumes an existing directory
            "board": {
                "squares": [SQUARES_X, SQUARES_Y],
                "square_len": p["square_len"],
                "marker_len": marker_len,
                "dict": DICT_NAME,
            },
            "samples": [],
        }
    )

    def take():
        old, end = last["frame"], time.time() + 2.0
        while last["frame"] is old:  # use a frame that arrives after Enter
            if time.time() > end:
                raise ValueError("no new image (check image_topic / info_topic)")
            time.sleep(0.01)
        msg, info = last["frame"]
        time.sleep(
            0.4
        )  # TF must reach the image stamp; the arm must still be there afterwards
        T = lookup(p["base_frame"], p["eef_frame"], Time.from_msg(msg.header.stamp))
        T_later = lookup(p["base_frame"], p["eef_frame"])
        move_mm, move_deg = (
            1000 * np.linalg.norm(T[:3, 3] - T_later[:3, 3]),
            angle(T[:3, :3], T_later[:3, :3]),
        )
        if move_mm > 1 or move_deg > 0.1:
            raise ValueError(f"arm moving ({move_mm:.1f} mm, {move_deg:.2f} deg)")

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _, _ = detector.detectBoard(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        n = 0 if ids is None else len(ids)
        if n < p["min_corners"]:
            raise ValueError(f"only {n} board corners (need {p['min_corners']})")
        obj, pts = board.matchImagePoints(corners, ids)
        K, D = np.array(info.k).reshape(3, 3), np.array(info.d)
        ok, rvec, tvec = cv2.solvePnP(obj, pts, K, D)
        rms = np.sqrt(
            ((cv2.projectPoints(obj, rvec, tvec, K, D)[0] - pts) ** 2).sum(-1).mean()
        )
        if not ok or rms > p["max_reproj_px"]:
            raise ValueError(f"reprojection {rms:.2f} px (max {p['max_reproj_px']})")

        if "frames" not in data:  # rigid chain, read once
            data["frames"] = {
                "T_eef_gripper": lookup(p["eef_frame"], GRIPPER).tolist(),
                "T_zedlink_optical": lookup(ZED_LINK, msg.header.frame_id).tolist(),
                "T_gripper_zedlink": lookup(
                    GRIPPER, ZED_LINK
                ).tolist(),  # current xacro value
            }
        name = f"{len(data['samples']):03d}.png"
        cv2.imwrite(str(out / name), img)
        data["samples"].append(
            {
                "T_base_eef": T.tolist(),
                "T_cam_target": mat(cv2.Rodrigues(rvec)[0], tvec.ravel()).tolist(),
                "corners": n,
                "rms_px": float(rms),
                "image": name,
            }
        )
        f.write_text(json.dumps(data, indent=1))
        Rs = [np.array(s["T_base_eef"])[:3, :3] for s in data["samples"]]
        spread = max(angle(a, b) for a in Rs for b in Rs)
        print(
            f"sample {len(Rs)}: {n} corners, {rms:.2f} px, max rotation between samples {spread:.0f} deg"
            + (" (WARN <30: tilt/roll more)" if spread < 30 else "")
        )

    print(
        f"board {SQUARES_X}x{SQUARES_Y}, square {p['square_len'] * 1000:.2f} mm, marker {marker_len * 1000:.2f} mm, {DICT_NAME}"
    )
    print(f"saving to {out}. Enter = sample, q + Enter = finish")
    while input().strip() != "q":
        try:
            take()
        except (ValueError, TransformException) as e:
            print(f"rejected: {e}")
    rclpy.shutdown()
    print(
        f"{len(data['samples'])} samples. Restore the arm: /xarm/set_mode {{data: 0}}, /xarm/set_state {{data: 0}}"
    )
    print(f"then: hand_eye_calibration.py solve {out}")


if __name__ == "__main__":
    if sys.argv[1:2] == ["capture"]:
        capture()
    elif sys.argv[1:2] == ["solve"] and len(sys.argv) > 2:
        solve(sys.argv[2])
    else:
        sys.exit(__doc__)
