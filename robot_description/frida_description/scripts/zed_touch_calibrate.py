#!/usr/bin/env python3
"""Calibración por *gripper-touch* de la traslación del montaje ZED (issue #944).

Corrige el residuo de traslación (~2-3 cm) del joint ``ZED`` (base_gripper -> zed)
en ``gripper_assem.xacro`` SIN tablero ArUco, usando la cinemática del brazo como
ground truth.  La orientación de la cámara ya está validada (180° en ``load_zed``),
así que aquí sólo resolvemos la TRASLACIÓN.

IDEA
----
Un punto físico fijo ``P`` se mide de dos formas:

* **TOQUE**: se toca ``P`` con la punta del gripper -> la FK del brazo da ``P`` en
  ``base_link`` (verdad de terreno, el brazo está calibrado a mm).
* **VISTA**: la cámara ve ``P``; en RViz (herramienta *Publish Point*) se clickea
  ``P`` sobre la nube -> el punto llega a ``base_link`` a través del árbol TF
  ACTUAL, es decir CON el error de montaje.

El error ``e = P_toque - P_vista`` (en ``base_link``) se debe a un offset
constante ``δ`` en el origin del joint ``ZED`` (expresado en ``base_gripper``).
Como el resto de la cadena ``zed -> ... -> óptico`` es rotación/traslación fija,
para cada muestra ``i`` tomada en una pose de visión distinta se cumple::

    P_toque = P_vista + R_{base_link<-base_gripper}(pose_i) · δ
    =>  δ_i = R_{base_link<-base_gripper}(pose_i)^T · (P_toque - P_vista)

``δ`` es invariante a la pose: se promedia sobre varias muestras y la dispersión
(std) mide la calidad.  El xyz corregido del joint es ``xyz_actual + δ``.

REQUISITOS EN VIVO
------------------
* Bringup real del xArm6 (robot_state_publisher con base_link..link_eef).
* Driver ZED publicando la nube; en RViz mostrarla con **Fixed Frame = base_link**
  y QoS Best Effort.  Debe verse a través del URDF (verificar que el árbol TF
  conecte ``base_link -> ... -> zed_camera_link -> óptico``).
* La herramienta *Publish Point* de RViz (publica en /clicked_point).

USO (dentro del contenedor home2-manipulation, sourced)
-------------------------------------------------------
    python3 zed_touch_calibrate.py \
        --tip-frame link_eef --tip-offset "0 0 0.18" \
        --xacro /workspace/src/robot_description/frida_description/urdf/Gripper/Custom/gripper_assem.xacro

Asistente: 't' tras tocar P con la punta; mueve a pose de visión, click en RViz
sobre P, 's'; repite >=5 puntos bien distribuidos (varía x/y/z y la orientación
del brazo); 'p' resuelve; 'w' escribe el xacro (con backup).
"""

import argparse
import shutil
import os
import re
import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

TF_ERRORS = (LookupException, ConnectivityException, ExtrapolationException)

# Pose CAD del joint ZED (gripper_assem.xacro). La orientación se conserva.
CAD_XYZ = "0 -0.0594045903873689 -0.0136964999847231"
ZED_RPY = "1.57079632679489 -1.5707963267949 0"


def quat_to_R(q):
    """q = (x, y, z, w) -> matriz de rotación 3x3."""
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class TouchCalib(Node):
    def __init__(self, args):
        super().__init__("zed_touch_calibrate")
        self.a = args
        self.tip_off = np.array(args.tip_offset, dtype=float)
        self.cur_xyz = np.array(args.current_zed_xyz, dtype=float)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.last_click = None      # PointStamped más reciente
        self.pending_touch = None   # np.array P_toque en base_link
        self.samples = []           # dicts(P_true, P_obs, e, delta)
        self.create_subscription(
            PointStamped, args.clicked_topic, self._on_click, 10
        )

    # --- callbacks -------------------------------------------------------
    def _on_click(self, msg):
        # Sin print aquí: imprimir desde el hilo del callback corrompe el prompt
        # de input() (mete "zed-calib> " en el buffer de lectura). El comando 's'
        # confirma qué click se usa.
        self.last_click = msg

    # --- TF --------------------------------------------------------------
    def _lookup(self, target, source):
        """Devuelve (t(3,), R(3x3)) de T_{target<-source} con la TF más reciente."""
        tf = self.buf.lookup_transform(
            target, source, Time(), timeout=Duration(seconds=1.0)
        )
        tr, ro = tf.transform.translation, tf.transform.rotation
        return (
            np.array([tr.x, tr.y, tr.z]),
            quat_to_R((ro.x, ro.y, ro.z, ro.w)),
        )

    # --- comandos --------------------------------------------------------
    def record_touch(self):
        try:
            t, R = self._lookup(self.a.base_frame, self.a.tip_frame)
        except TF_ERRORS as e:
            print(f"  ! TF {self.a.base_frame}<-{self.a.tip_frame} no disponible: {e}")
            return
        P_true = R @ self.tip_off + t
        self.pending_touch = P_true
        self.last_click = None  # exigir click FRESCO tras el toque (evita usar uno viejo)
        print(
            f"  TOQUE: punta ({self.a.tip_frame}+offset {self.tip_off.tolist()}) "
            f"en {self.a.base_frame} = [{P_true[0]:.4f}, {P_true[1]:.4f}, {P_true[2]:.4f}] m"
        )
        for fr in (self.a.tip_frame, self.a.gripper_frame):
            try:
                tt, _ = self._lookup(self.a.base_frame, fr)
                print(f"     (ref {fr:14s} = [{tt[0]:.4f}, {tt[1]:.4f}, {tt[2]:.4f}])")
            except TF_ERRORS:
                pass

    def record_see(self):
        if self.pending_touch is None:
            print("  ! primero registra un TOQUE ('t').")
            return
        if self.last_click is None:
            print("  ! sin /clicked_point. En RViz usa 'Publish Point' sobre P en la nube.")
            return
        c = self.last_click
        print(f"  usando click {c.header.frame_id} ({c.point.x:.3f}, {c.point.y:.3f}, {c.point.z:.3f})")
        p = np.array([c.point.x, c.point.y, c.point.z])
        if c.header.frame_id and c.header.frame_id != self.a.base_frame:
            try:
                t, R = self._lookup(self.a.base_frame, c.header.frame_id)
                p = R @ p + t
            except TF_ERRORS as e:
                print(f"  ! no pude transformar click {c.header.frame_id}->{self.a.base_frame}: {e}")
                return
        try:
            _, R_bg = self._lookup(self.a.base_frame, self.a.gripper_frame)
        except TF_ERRORS as e:
            print(f"  ! TF {self.a.base_frame}<-{self.a.gripper_frame} no disponible: {e}")
            return
        P_true = self.pending_touch
        e = P_true - p           # error en base_link
        delta = R_bg.T @ e       # corrección en frame base_gripper
        self.samples.append(dict(P_true=P_true, P_obs=p, e=e, delta=delta))
        n = len(self.samples)
        print(f"  VISTA muestra #{n}:")
        print(f"     P_vista(cám) {self.a.base_frame} = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]")
        print(f"     e = toque-vista          = [{e[0]:+.4f}, {e[1]:+.4f}, {e[2]:+.4f}] (|e|={np.linalg.norm(e) * 100:.1f} cm)")
        print(f"     δ (base_gripper)         = [{delta[0]:+.4f}, {delta[1]:+.4f}, {delta[2]:+.4f}]")
        self.pending_touch = None
        self.last_click = None

    def solve(self):
        if not self.samples:
            print("  ! sin muestras.")
            return None
        D = np.array([s["delta"] for s in self.samples])
        E = np.array([s["e"] for s in self.samples])
        dmean, dstd, emean = D.mean(0), D.std(0), E.mean(0)
        new = self.cur_xyz + dmean
        disp = "OK" if dstd.max() < 0.005 else "ALTA dispersión -> revisar toques/clicks o punta"
        print("\n========= RESULTADO calibración gripper-touch =========")
        print(f"  n muestras            : {len(self.samples)}")
        print(f"  e medio ({self.a.base_frame})  : [{emean[0]:+.4f}, {emean[1]:+.4f}, {emean[2]:+.4f}] m  (cámara vs verdad; z={emean[2] * 100:+.1f} cm)")
        print(f"  δ medio (base_gripper): [{dmean[0]:+.5f}, {dmean[1]:+.5f}, {dmean[2]:+.5f}] m")
        print(f"  δ std (consistencia)  : [{dstd[0]:.5f}, {dstd[1]:.5f}, {dstd[2]:.5f}] m  ({disp})")
        print(f"  xyz ACTUAL joint ZED  : {self.cur_xyz[0]:.10g} {self.cur_xyz[1]:.10g} {self.cur_xyz[2]:.10g}")
        print(f"  xyz NUEVO  joint ZED  : {new[0]:.10g} {new[1]:.10g} {new[2]:.10g}")
        print("  -> línea para gripper_assem.xacro (joint name=\"ZED\"):")
        print(f'     <origin xyz="{new[0]:.10g} {new[1]:.10g} {new[2]:.10g}" rpy="{ZED_RPY}" />')
        print("=======================================================\n")
        return new

    def write_xacro(self):
        new = self.solve()
        if new is None:
            return
        path = self.a.xacro
        if not path or not os.path.exists(path):
            print(f"  ! sin --xacro válido (path='{path}'). Sólo se imprimió el resultado.")
            return
        with open(path) as f:
            txt = f.read()
        m = re.search(r'(name="ZED".*?<origin\s+xyz=")([^"]*)(")', txt, re.S)
        if not m:
            print("  ! no encontré el joint ZED/origin en el xacro.")
            return
        shutil.copy(path, path + ".bak")
        newxyz = f"{new[0]:.10g} {new[1]:.10g} {new[2]:.10g}"
        txt2 = txt[: m.start(2)] + newxyz + txt[m.end(2):]
        with open(path, "w") as f:
            f.write(txt2)
        print(f"  escrito {path}\n  backup en {path}.bak\n  joint ZED xyz -> {newxyz}\n  reconstruye/relanza para probar.")


def repl(node):
    h = ("comandos: t=toque(FK)  s=ver(usa último click)  p=resolver  "
         "w=escribir xacro  u=deshacer  l=listar  h=ayuda  q=salir")
    print("\n" + h)
    while rclpy.ok():
        try:
            cmd = input("zed-calib> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd in ("q", "quit", "exit"):
            break
        elif cmd in ("t", "touch", "toque"):
            node.record_touch()
        elif cmd in ("s", "see", "ver"):
            node.record_see()
        elif cmd in ("p", "solve", "print"):
            node.solve()
        elif cmd in ("w", "write"):
            node.write_xacro()
        elif cmd in ("u", "undo"):
            if node.samples:
                node.samples.pop()
                print(f"  deshecho; quedan {len(node.samples)}")
            else:
                print("  (vacío)")
        elif cmd in ("l", "list"):
            for i, s in enumerate(node.samples, 1):
                e = s["e"]
                print(f"  #{i} e=[{e[0]:+.3f},{e[1]:+.3f},{e[2]:+.3f}] |e|={np.linalg.norm(e) * 100:.1f}cm")
        elif cmd in ("h", "help", "?"):
            print(h)
        elif cmd:
            print("  ? " + h)


def main():
    p = argparse.ArgumentParser(description="Calibración gripper-touch del montaje ZED (#944)")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--tip-frame", default="link_eef")
    p.add_argument("--tip-offset", default="0 0 0",
                   help='offset en tip_frame "x y z" (m) hasta el punto de contacto físico')
    p.add_argument("--gripper-frame", default="base_gripper")
    p.add_argument("--clicked-topic", default="/clicked_point")
    p.add_argument("--current-zed-xyz", default=CAD_XYZ,
                   help="xyz actual del joint ZED en gripper_assem.xacro")
    p.add_argument("--xacro", default="",
                   help="ruta a gripper_assem.xacro para 'w' (escribir)")
    a = p.parse_args()
    a.tip_offset = [float(x) for x in a.tip_offset.split()]
    a.current_zed_xyz = [float(x) for x in a.current_zed_xyz.split()]

    rclpy.init()
    node = TouchCalib(a)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f"[init] base={a.base_frame} tip={a.tip_frame}+{a.tip_offset} "
          f"gripper={a.gripper_frame} click={a.clicked_topic}")
    print(f"[init] xyz actual joint ZED = {a.current_zed_xyz}")
    try:
        repl(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
