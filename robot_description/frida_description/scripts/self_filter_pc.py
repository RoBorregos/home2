#!/usr/bin/env python3
"""Self-filter del cuerpo del robot en la point cloud (issue #944, paso 2).

Con la ZED eye-in-hand, la cámara capta el propio cuerpo del robot (gripper,
dedos, codo) y esos puntos contaminan el octomap (voxeles fantasma) y a GPD
(grasps sobre el brazo). Este nodo quita esos puntos ANTES de que la nube llegue
a sus consumidores.

MÉTODO (canónico, linaje robot_self_filter/robot_body_filter): containment contra
la geometría de colisión del robot. Aquí usamos **primitivas (cápsulas por link)**
en vez de las mallas STL — la literatura (robot_body_filter) recomienda primitivas
porque el containment con malla es mucho más lento (crítico en el Jetson, con el
cuerpo siempre en el FOV). Cada link se modela como una cápsula (segmento entre dos
frames + radio inflado); un punto se descarta si cae dentro de CUALQUIER cápsula.

Las poses de los links se leen por TF (FK del robot_state_publisher), así que el
filtro sigue al brazo en cualquier configuración.

USO (en el contenedor, sourced). Para PROBAR sin tocar el pipeline:
  python3 self_filter_pc.py --in /point_cloud --out /point_cloud_self_filtered
y comparar en RViz /point_cloud (con cuerpo) vs /point_cloud_self_filtered (limpio).
Para PRODUCCIÓN (tras validar): --in /point_cloud_raw --out /point_cloud.
"""

import argparse
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

TF_ERRORS = (LookupException, ConnectivityException, ExtrapolationException)
O = [0.0, 0.0, 0.0]

# Cápsulas (cuerpo del robot): ((frame_a, offset_a), (frame_b, offset_b), radio[m]).
# offset en el frame local (m). Radios = grosor del link inflado; dedos/ZED (lo más
# cercano a la cámara) cubiertos generosamente. Tunables con --inflate.
CAPSULES = [
    # Radios GENEROSOS: la malla del xArm6 se separa del eje articular, así que las
    # cápsulas finas dejaban puntos de superficie fuera. Engordadas para cubrir la
    # envolvente del brazo+gripper (quitar una cáscara alrededor del robot es seguro:
    # ahí nunca queremos voxeles, y el objeto en la mesa está lejos del eje del brazo).
    (("link_base", O), ("link1", O), 0.13),
    (("link1", O), ("link2", O), 0.13),
    (("link2", O), ("link3", O), 0.11),
    (("link3", O), ("link4", O), 0.10),
    (("link4", O), ("link5", O), 0.10),
    (("link5", O), ("link6", O), 0.10),
    (("link6", O), ("link_eef", O), 0.11),
    (("link_eef", O), ("base_gripper", O), 0.12),
    # Dedos hasta la punta (malla del dedo: +0.102 en z del frame del dedo)
    (("base_gripper", O), ("leftFinger", [0.0, 0.0, 0.102]), 0.09),
    (("base_gripper", O), ("rightFinger", [0.0, 0.0, 0.102]), 0.09),
    # Cuerpo de la ZED (ancho ~0.175 m)
    (("base_gripper", O), ("zed", O), 0.14),
]


def quat_to_R(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def seg_dist_sq(P, a, b):
    """Distancia^2 punto-segmento, vectorizada. P:(N,3), a,b:(3,)."""
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-9:
        d = P - a
        return np.einsum("ij,ij->i", d, d)
    t = np.clip(((P - a) @ ab) / L2, 0.0, 1.0)
    proj = a + np.outer(t, ab)
    d = P - proj
    return np.einsum("ij,ij->i", d, d)


class SelfFilter(Node):
    def __init__(self, args):
        super().__init__("self_filter_pc")
        self.args = args
        self.buf = Buffer()
        # spin_thread=True: el listener de TF corre en su PROPIO hilo, así el buffer
        # se llena aunque el callback pesado de la nube acapare el executor principal
        # (si no, el TF se starva -> "unconnected trees" -> removidos=0).
        self.tl = TransformListener(self.buf, self, spin_thread=True)
        self.pub = self.create_publisher(PointCloud2, args.out, qos_profile_sensor_data)
        self.pub_removed = self.create_publisher(PointCloud2, args.removed, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, args.inp, self.cb, qos_profile_sensor_data)
        self.n = 0
        self.get_logger().info(
            f"self_filter: {args.inp} -> {args.out}  inflate={args.inflate}m  "
            f"{len(CAPSULES)} capsulas")

    def _ep(self, cloud_frame, frame, offset, stamp):
        """Posición (3,) de (frame+offset) en el frame de la nube, vía TF."""
        try:
            tf = self.buf.lookup_transform(cloud_frame, frame, stamp,
                                           timeout=Duration(seconds=0.05))
        except TF_ERRORS:
            tf = self.buf.lookup_transform(cloud_frame, frame, rclpy.time.Time(),
                                           timeout=Duration(seconds=0.2))
        tr, ro = tf.transform.translation, tf.transform.rotation
        t = np.array([tr.x, tr.y, tr.z])
        if offset == O:
            return t
        R = quat_to_R((ro.x, ro.y, ro.z, ro.w))
        return R @ np.array(offset, dtype=float) + t

    def cb(self, msg):
        cloud_frame = msg.header.frame_id
        # endpoints de cada cápsula en el frame de la nube (cache por frame)
        caps = []
        cache = {}
        ok = True
        for (fa, oa), (fb, ob), r in CAPSULES:
            try:
                ka, kb = (fa, tuple(oa)), (fb, tuple(ob))
                if ka not in cache:
                    cache[ka] = self._ep(cloud_frame, fa, oa, msg.header.stamp)
                if kb not in cache:
                    cache[kb] = self._ep(cloud_frame, fb, ob, msg.header.stamp)
                caps.append((cache[ka], cache[kb], r + self.args.inflate))
            except TF_ERRORS as e:
                ok = False
                self.get_logger().warn(f"TF {cloud_frame}<-{fa}/{fb} no disp: {e}", throttle_duration_sec=5.0)
        if not ok and not caps:
            self.pub.publish(msg)  # sin TF: pasa la nube tal cual
            return

        rec = pc2.read_points(msg, skip_nans=False)  # array estructurado, todos los campos
        if rec.shape[0] == 0:
            self.pub.publish(msg)
            return
        P = np.stack([rec["x"], rec["y"], rec["z"]], axis=-1).astype(np.float64)
        finite = np.isfinite(P).all(axis=1)
        remove = np.zeros(P.shape[0], dtype=bool)
        # Recorte de campo cercano (cloud en frame de la cámara -> origen = cámara):
        # el gripper está más cerca que la profundidad mínima de la ZED, así que sus
        # puntos se PROYECTAN a ~min-depth ("volando", no sobre el gripper real). El
        # filtro por cuerpo no los caza; este recorte sí (objeto/mesa están más lejos).
        if self.args.min_range > 0.0:
            remove |= np.einsum("ij,ij->i", P, P) < (self.args.min_range ** 2)
        for a, b, r in caps:
            remove |= (seg_dist_sq(P, a, b) < r * r)
        keep = (~remove) & finite
        kept = rec[keep]
        self.pub.publish(pc2.create_cloud(msg.header, msg.fields, kept.tolist()))
        rem = rec[remove & finite]              # DEBUG: puntos quitados (deben caer sobre el robot)
        if rem.shape[0] > 0:
            self.pub_removed.publish(pc2.create_cloud(msg.header, msg.fields, rem.tolist()))
        self.n += 1
        if self.n % 30 == 0:
            self.get_logger().info(
                f"in={P.shape[0]} out={int(keep.sum())} removidos={int(remove.sum())}")


def main():
    p = argparse.ArgumentParser(description="Self-filter del cuerpo del robot (#944)")
    p.add_argument("--in", dest="inp", default="/point_cloud_raw")
    p.add_argument("--out", default="/point_cloud")
    p.add_argument("--removed", default="/point_cloud_self_removed", help="DEBUG: puntos quitados")
    p.add_argument("--inflate", type=float, default=0.03, help="margen extra de radio (m)")
    p.add_argument("--min-range", type=float, default=0.35,
                   help="quita puntos a < esta dist de la camara (m); el gripper near-field se proyecta aqui")
    a = p.parse_args(sys.argv[1:])
    rclpy.init()
    node = SelfFilter(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
