#!/usr/bin/env python3
"""Captura UN frame de la ZED para validar el ROI (#944).

Guarda RGB + depth (+ conteo de la nube) como arrays .npy crudos en /tmp/zed_capture/
para renderizarlos/leerlos en otra máquina (no depende de cv2/PIL en el contenedor).

OBJETIVO: comparar DÓNDE aparece el gripper en la imagen (RGB) vs QUÉ región queda
con profundidad válida (depth) — el ROI aplicado a depth deja en NaN/0 la zona recortada.
Con el ROI desactivado (estado actual) el gripper saldrá entero en el depth = baseline.

Auto-detecta los topics; se pueden forzar con --rgb / --depth / --cloud.
Uso (en el contenedor home2-zed o cualquiera con los topics visibles):
  python3 zed_capture.py
  python3 zed_capture.py --rgb /zed/zed_node/rgb/image_rect_color --depth /zed/zed_node/depth/depth_registered
"""
import os
import sys
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

OUT = "/tmp/zed_capture"


def decode_image(msg):
    """msg.data -> ndarray, respetando step (padding de fila) y encoding."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        ch, dt = 3, np.uint8
    elif enc in ("rgba8", "bgra8"):
        ch, dt = 4, np.uint8
    elif enc in ("mono8",):
        ch, dt = 1, np.uint8
    elif enc in ("32fc1", "32FC1".lower()):
        ch, dt = 1, np.float32
    elif enc in ("16uc1",):
        ch, dt = 1, np.uint16
    else:
        ch, dt = None, None
    if dt is None:
        return None, enc
    bpp = np.dtype(dt).itemsize * ch                    # bytes por pixel
    rows = buf.reshape(msg.height, msg.step)            # step = bytes por fila (con padding)
    usable = np.ascontiguousarray(rows[:, : msg.width * bpp])  # (h, width*bpp) uint8
    arr = usable.view(dt).reshape(msg.height, msg.width, ch)   # view a dtype, luego forma
    if ch == 1:
        arr = arr.reshape(msg.height, msg.width)
    return np.ascontiguousarray(arr), enc


class Cap(Node):
    def __init__(self, a):
        super().__init__("zed_capture")
        self.a = a
        self.got = {"rgb": False, "depth": False, "cloud": False}
        names_types = self.get_topic_names_and_types()
        topics = {n: t for n, t in names_types}

        def pick(want_type, prefer, avoid=()):
            cands = [n for n, ts in topics.items() if any(want_type in t for t in ts)]
            for kw in prefer:
                for n in cands:
                    if kw in n and not any(x in n for x in avoid):
                        return n
            return cands[0] if cands else None

        self.t_rgb = a.rgb or pick("Image", ["rgb/image_rect_color", "rgb", "left/image_rect", "left/image"], avoid=("depth", "raw_gray"))
        self.t_depth = a.depth or pick("Image", ["depth/depth_registered", "depth_registered", "depth"], avoid=("confidence",))
        self.t_cloud = a.cloud or pick("PointCloud2", ["point_cloud", "cloud_registered", "points"])
        print(f"RGB   : {self.t_rgb}\nDEPTH : {self.t_depth}\nCLOUD : {self.t_cloud}\n" + "-" * 50)

        if self.t_rgb:
            self.create_subscription(Image, self.t_rgb, self.cb_rgb, qos_profile_sensor_data)
        if self.t_depth:
            self.create_subscription(Image, self.t_depth, self.cb_depth, qos_profile_sensor_data)
        if self.t_cloud:
            self.create_subscription(PointCloud2, self.t_cloud, self.cb_cloud, qos_profile_sensor_data)

    def cb_rgb(self, msg):
        if self.got["rgb"]:
            return
        arr, enc = decode_image(msg)
        if arr is None:
            print(f"RGB encoding no soportado: {enc}"); self.got["rgb"] = True; return
        np.save(f"{OUT}/rgb.npy", arr)
        with open(f"{OUT}/rgb_meta.txt", "w") as f:
            f.write(f"topic={self.t_rgb} enc={enc} shape={arr.shape} frame={msg.header.frame_id}\n")
        print(f"RGB   guardado {arr.shape} enc={enc}")
        self.got["rgb"] = True

    def cb_depth(self, msg):
        if self.got["depth"]:
            return
        arr, enc = decode_image(msg)
        if arr is None:
            print(f"DEPTH encoding no soportado: {enc}"); self.got["depth"] = True; return
        d = arr.astype(np.float32)
        np.save(f"{OUT}/depth.npy", d)
        finite = np.isfinite(d) & (d > 0)
        with open(f"{OUT}/depth_meta.txt", "w") as f:
            f.write(f"topic={self.t_depth} enc={enc} shape={d.shape} frame={msg.header.frame_id} "
                    f"valid_px={int(finite.sum())}/{d.size} "
                    f"range=[{np.nanmin(d[finite]) if finite.any() else 0:.3f},"
                    f"{np.nanmax(d[finite]) if finite.any() else 0:.3f}]\n")
        print(f"DEPTH guardado {d.shape} enc={enc} valid={int(finite.sum())}/{d.size}")
        self.got["depth"] = True

    def cb_cloud(self, msg):
        if self.got["cloud"]:
            return
        with open(f"{OUT}/cloud_meta.txt", "w") as f:
            f.write(f"topic={self.t_cloud} frame={msg.header.frame_id} "
                    f"w={msg.width} h={msg.height} points={msg.width*msg.height}\n")
        print(f"CLOUD info w={msg.width} h={msg.height} frame={msg.header.frame_id}")
        self.got["cloud"] = True

    def done(self):
        return all(self.got[k] for k in self.got if getattr(self, f"t_{k}"))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--rgb"); p.add_argument("--depth"); p.add_argument("--cloud")
    p.add_argument("--timeout", type=float, default=15.0)
    a = p.parse_args()
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    n = Cap(a)
    import time
    end = None
    # spin hasta tener todo o timeout (sin time.time(): contamos iteraciones)
    iters = int(a.timeout / 0.1)
    for _ in range(iters):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.done():
            break
    print("-" * 50)
    print("OK" if n.done() else "TIMEOUT (faltaron topics)")
    print(f"Artefactos en {OUT}/  (rgb.npy, depth.npy, *_meta.txt)")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
