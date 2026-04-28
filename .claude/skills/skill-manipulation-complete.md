# Skill: Manipulación y Percepción 3D — Pipeline Completa para RoboCup@Home
# Robot: Frida (xArm6 + RGBD cameras + parallel jaw gripper)
# Basado en investigación de documentación oficial, papers, y mejores prácticas 2025-2026

---

## 1. HARDWARE: xArm6 (UFactory)

### Especificaciones del brazo
- DOF: 6
- Alcance máximo: 700mm
- Payload máximo: 5kg
- Repetibilidad: ±0.1mm
- Velocidad máxima de joints: 180°/s
- Comunicación: Ethernet TCP/IP al control box
- Firmware: xArm SDK (C++), wrapper ROS2 vía `xarm_ros2`
- Modos de operación:
  - Mode 0: Position control (default, usado para MoveIt2)
  - Mode 1: Servo mode (low latency, útil para visual servoing)
  - Mode 4: Joint velocity control
  - Mode 5: Cartesian velocity control
  - Mode 6: Joint online trajectory planning (suavizado built-in)

### Joint Limits (radianes, del URDF oficial)
| Joint | Min (rad) | Max (rad) | Max Vel (rad/s) |
|-------|-----------|-----------|-----------------|
| J1 | -6.2832 | 6.2832 | 3.14 |
| J2 | -2.0594 | 2.0944 | 3.14 |
| J3 | -0.1920 | 3.9270 | 3.14 |
| J4 | -6.2832 | 6.2832 | 3.14 |
| J5 | -1.6930 | 3.1416 | 3.14 |
| J6 | -6.2832 | 6.2832 | 3.14 |

### Gripper (UFactory parallel jaw)
- Tipo: Parallel jaw, 2 dedos
- Apertura máxima: ~85mm (0.085m)
- Fuerza de agarre: ajustable, ~20-40N
- Control: vía xarm_msgs/srv/SetInt16 (GPIO) o topic de gripper
- Convención en xarm_ros2:
  - `open_gripper()`: abre completamente
  - `close_gripper()`: cierra con fuerza
  - `set_gripper_position(pos)`: pos 0 (cerrado) a 850 (abierto max)
- `drive_joint`: joint virtual del gripper, indica apertura actual
  - 0.0 = completamente cerrado
  - ~0.85 = completamente abierto
  - ATENCIÓN: la convención puede variar según la versión del driver

### Frames TF importantes
```
base_link
  └── link1 (J1)
       └── link2 (J2)
            └── link3 (J3)
                 └── link4 (J4)
                      └── link5 (J5)
                           └── link6 (J6)
                                └── link_eef (flange)
                                     ├── gripper_base
                                     │    ├── left_finger
                                     │    └── right_finger
                                     └── zed2_camera_center        ← ZED 2 montada en el gripper
                                          ├── zed2_left_camera_frame
                                          ├── zed2_right_camera_frame
                                          └── zed2_imu_link
```
- `link_eef`: punto de referencia para end-effector en MoveIt2
- `zed2_camera_center`: frame principal de la ZED 2, SE MUEVE con el brazo (eye-in-hand)
- `zed2_left_camera_frame`: frame óptico de la cámara izquierda (referencia para depth y point cloud)
- La relación `link_eef → zed2_camera_center` es un TF estático definido en el URDF/xacro
- SIEMPRE verificar TF completa: `ros2 run tf2_tools view_frames`
- **CRÍTICO**: si esta TF estática está mal calibrada, TODOS los grasps estarán desplazados

### Integración xarm_ros2 en ROS2 Humble
```bash
# Launch del driver real
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117

# Launch con MoveIt2
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.117 \
  add_gripper:=true
```

Servicios clave del driver:
- `/xarm/set_mode` — cambiar modo de operación
- `/xarm/set_state` — 0=ready, 3=pause, 4=stop
- `/xarm/motion_enable` — habilitar/deshabilitar joints
- `/xarm/set_servo_angle` — mover a posición de joints
- `/xarm/set_position` — mover a pose cartesiana (mm, rad)
- `/xarm/get_err_warn_code` — verificar errores del controller
- `/xarm/clean_error` — limpiar errores

Regla: SIEMPRE verificar error code después de un movimiento fallido:
```python
err_code = call_service('/xarm/get_err_warn_code')
if err_code.err != 0:
    call_service('/xarm/clean_error')
    call_service('/xarm/motion_enable', id=8, data=1)
    call_service('/xarm/set_mode', data=0)
    call_service('/xarm/set_state', data=0)
```

---

## 2. PERCEPCIÓN 3D (Perception3D Pipeline)

### 2.1 Cámara: ZED 2 (Stereolabs) — Montaje Eye-in-Hand

La ZED 2 está montada en un lado del gripper (eye-in-hand), NO fija en el cuerpo del robot.

**Especificaciones ZED 2:**
- Tipo: Cámara estéreo pasiva (no proyector IR)
- Resolución: hasta 2K (2208×1242) por sensor
- Depth engine: Neural depth (redes neuronales para stereo matching)
- FOV: **110° (H) × 70° (V) × 120° (D)** — mucho más amplio que RealSense
- Rango de depth: 0.3m a 20m (indoor, HD1080)
- Precisión depth: ~1% del rango a distancias cortas
- Baseline: 120mm entre lentes
- IMU integrado: acelerómetro + giróscopo (útil para compensar movimiento)
- Sensores adicionales: barómetro, magnetómetro, sensor de temperatura
- Conexión: USB 3.0 Type-C
- Requiere: GPU NVIDIA con CUDA para el ZED SDK

**Topics ROS2 principales (zed-ros2-wrapper):**
```
/zed/zed_node/rgb/image_rect_color           # Image color rectificada (sensor_msgs/Image)
/zed/zed_node/depth/depth_registered          # Depth map 32-bit float en metros (sensor_msgs/Image)
/zed/zed_node/point_cloud/cloud_registered    # Point cloud XYZRGBA (sensor_msgs/PointCloud2)
/zed/zed_node/confidence/confidence_map       # Mapa de confianza del depth
/zed/zed_node/disparity/disparity_image       # Mapa de disparidad
/zed/zed_node/left/image_rect_color           # Imagen izquierda rectificada
/zed/zed_node/right/image_rect_color          # Imagen derecha rectificada
/zed/zed_node/imu/data                        # Datos del IMU (sensor_msgs/Imu)
```

**Launch del nodo ZED 2:**
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

**Configuración recomendada para manipulación (`zed2.yaml`):**
```yaml
depth:
  depth_mode: NEURAL            # Mejor calidad que ULTRA para close-range
  min_depth: 0.3                # 30cm mínimo (objetos cercanos al gripper)
  max_depth: 3.0                # 3m suficiente para tabletop
  depth_stabilization: 1        # Estabilización temporal (1=mínima, 100=máxima)
  quality: QUALITY              # PERFORMANCE, QUALITY, o ULTRA

point_cloud:
  point_cloud_freq: 15.0        # Hz — balance entre frescura y CPU
  
general:
  resolution: HD720             # 720p es buen balance para manipulation
  grab_frame_rate: 30           # FPS

region_of_interest:
  automatic_roi: true           # Automáticamente excluye partes del robot visibles
```

### 2.1.1 Implicaciones de Eye-in-Hand

**Ventajas:**
- El punto de vista se mueve con el brazo → puede observar objetos desde múltiples ángulos
- Puede acercarse al objeto para mejor resolución de depth
- Puede mirar "hacia abajo" directamente sobre la mesa (ideal para grasping)
- Permite visual servoing (ajustar grasp en tiempo real con feedback visual)

**Desafíos y reglas críticas:**

1. **TF dinámico**: El frame de la cámara (`zed2_camera_link`) se mueve con el end-effector.
   La cadena TF es: `base_link → link1 → ... → link6 → link_eef → zed2_camera_link`
   Los point clouds se transforman automáticamente a `base_link` vía TF, pero SOLO si la cadena TF es correcta y actualizada.

2. **NUNCA capturar point cloud mientras el brazo se mueve**: el motion blur y el TF desactualizado corrompen el point cloud. SIEMPRE:
   ```python
   # 1. Mover brazo a pose de observación
   moveit2.move_to_configuration(STARE_POSE)
   moveit2.wait_until_executed()
   
   # 2. Esperar estabilización (brazo + cámara + depth)
   time.sleep(1.0)  # Mínimo 0.5s, 1.0s recomendado
   
   # 3. AHORA capturar point cloud
   cloud = get_latest_point_cloud()
   ```

3. **La cámara puede ver el gripper**: Cuando el brazo está en ciertas poses, los dedos del gripper aparecen en la imagen/point cloud.
   - El ZED SDK tiene `automatic_roi: true` que detecta y excluye partes del robot
   - Alternativamente: filtrar puntos que estén dentro del bounding box conocido del gripper
   - O: usar un crop agresivo de PassThrough que solo incluya la zona de la mesa

4. **Oclusión por el propio brazo**: En eye-in-hand, el brazo puede ocultar partes de la escena.
   - Solución: elegir poses de observación donde el brazo no tape los objetos
   - Tener 2-3 poses de observación predefinidas para diferentes zonas de la mesa

5. **Camera viewpoint para GPD**: Como la cámara se mueve, el viewpoint para GPD cambia con cada captura.
   - Si el point cloud se transforma a `base_link`, el viewpoint es la posición de la cámara EN base_link al momento de captura
   - Calcular dinámicamente: `viewpoint = tf_buffer.lookup_transform('base_link', 'zed2_camera_link', time)`
   ```python
   # Obtener posición actual de la cámara para GPD
   try:
       cam_tf = tf_buffer.lookup_transform('base_link', 'zed2_camera_link', rclpy.time.Time())
       viewpoint = [
           cam_tf.transform.translation.x,
           cam_tf.transform.translation.y,
           cam_tf.transform.translation.z
       ]
   except Exception as e:
       logger.error(f"Cannot get camera TF: {e}")
       viewpoint = [0, 0, 0]  # Fallback — NO ideal, grasps tendrán perspectiva incorrecta
   ```

6. **Depth mínimo de 0.3m**: La ZED 2 no puede medir depth a menos de 30cm. Si el gripper está a 20cm del objeto, NO habrá depth data. Planificar las poses de observación a al menos 35-40cm de distancia de los objetos.

7. **Ventaja: Object Detection built-in**: El ZED SDK incluye detección de objetos integrada (YOLO-based):
   ```bash
   ros2 service call /zed/zed_node/enable_obj_det std_srvs/srv/SetBool "{data: true}"
   ```
   Topics: `/zed/zed_node/obj_det/objects` — incluye bounding box 3D + tracking ID + label

**QoS para topics de ZED:**
```python
from rclpy.qos import qos_profile_sensor_data
# Usar BEST_EFFORT para todos los topics de imagen y point cloud
self.sub = self.create_subscription(PointCloud2, topic, callback, qos_profile_sensor_data)
```

### 2.2 Pipeline de procesamiento de point cloud (PCL)

La pipeline estándar para tabletop manipulation:

```
Raw PointCloud2
    │
    ▼
[1] Voxel Grid Downsample    ← Reduce resolución, elimina NaN
    │
    ▼
[2] PassThrough Filters       ← Crop ROI (x, y, z)
    │
    ▼
[3] RANSAC Plane Segmentation ← Detectar y remover superficie plana (mesa)
    │
    ▼
[4] Statistical Outlier Removal ← Quitar ruido
    │
    ▼
[5] Euclidean Cluster Extraction ← Separar objetos individuales
    │
    ▼
[6] Per-cluster processing     ← Centroide, bounding box, clasificación
```

#### Paso 1: Voxel Grid Downsampling
Reduce la densidad del point cloud manteniendo la estructura.

```cpp
pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
voxel.setInputCloud(cloud);
voxel.setLeafSize(0.005f, 0.005f, 0.005f);  // 5mm — balance entre precisión y velocidad
voxel.filter(*cloud_filtered);
```
- Leaf size 0.005m (5mm): bueno para objetos de cocina
- Leaf size 0.01m (1cm): más rápido, suficiente para objetos grandes
- NUNCA usar leaf size mayor a 0.02m para grasping — pierde detalle

#### Paso 2: PassThrough Filters
Crop a la región de interés. Aplicar en los 3 ejes:

```cpp
// Filtro Z (altura): solo lo que está arriba de la mesa
pcl::PassThrough<pcl::PointXYZRGB> pass_z;
pass_z.setInputCloud(cloud_filtered);
pass_z.setFilterFieldName("z");
pass_z.setFilterLimits(table_height + 0.01, table_height + 0.50);  // 1cm arriba de mesa, hasta 50cm
pass_z.filter(*cloud_filtered);

// Filtro X (profundidad): alcance del brazo
pcl::PassThrough<pcl::PointXYZRGB> pass_x;
pass_x.setFilterFieldName("x");
pass_x.setFilterLimits(0.1, 0.75);  // 10cm min (evitar cámara), 75cm max (alcance brazo)
pass_x.filter(*cloud_filtered);

// Filtro Y (lateral)
pcl::PassThrough<pcl::PointXYZRGB> pass_y;
pass_y.setFilterFieldName("y");
pass_y.setFilterLimits(-0.4, 0.4);
pass_y.filter(*cloud_filtered);
```

REGLA: Los límites de PassThrough DEBEN ser parámetros ROS2, NO hardcoded. Cambian entre arenas.

#### Paso 3: RANSAC Plane Segmentation
Detecta la superficie plana más grande (mesa) y la remueve:

```cpp
pcl::SACSegmentation<pcl::PointXYZRGB> seg;
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.01);    // 1cm — tolerancia al plano
seg.setMaxIterations(1000);

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
seg.setInputCloud(cloud_filtered);
seg.segment(*inliers, *coefficients);

// Extraer puntos que NO son el plano (los objetos)
pcl::ExtractIndices<pcl::PointXYZRGB> extract;
extract.setInputCloud(cloud_filtered);
extract.setIndices(inliers);
extract.setNegative(true);  // true = extraer lo que NO es el plano
extract.filter(*cloud_objects);
```

Los `coefficients` del plano (ax + by + cz + d = 0) se usan para:
- Determinar la altura de la mesa
- Crear collision object del plano en MoveIt2
- Calcular la normal de la superficie para grasps top-down

#### Paso 4: Statistical Outlier Removal
Quita puntos aislados que son ruido:

```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
sor.setInputCloud(cloud_objects);
sor.setMeanK(50);             // neighbors a considerar
sor.setStddevMulThresh(1.0);  // multiplicador de std dev
sor.filter(*cloud_clean);
```

#### Paso 5: Euclidean Cluster Extraction
Separa objetos individuales:

```cpp
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
tree->setInputCloud(cloud_clean);

std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
ec.setClusterTolerance(0.02);    // 2cm — distancia entre puntos del mismo cluster
ec.setMinClusterSize(100);       // mínimo 100 puntos (evita ruido)
ec.setMaxClusterSize(25000);     // máximo 25000 puntos
ec.setSearchMethod(tree);
ec.setInputCloud(cloud_clean);
ec.extract(cluster_indices);
```

Tuning de `ClusterTolerance`:
- 0.01m (1cm): separa objetos muy juntos pero puede fragmentar objetos grandes
- 0.02m (2cm): buen balance para objetos de cocina
- 0.03m (3cm): une objetos que están cerca, útil si los objetos son porosos

#### Paso 6: Per-cluster Processing
Para cada cluster extraído:

```cpp
for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : indices.indices)
        cluster->push_back((*cloud_clean)[idx]);

    // Centroide
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // Bounding box (axis-aligned)
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    // Dimensiones del objeto
    float width  = max_pt.x - min_pt.x;
    float depth  = max_pt.y - min_pt.y;
    float height = max_pt.z - min_pt.z;

    // Publicar como cluster individual para grasping
}
```

### 2.3 Consideraciones de performance
- La pipeline completa debe ejecutar en < 500ms para ser usable en competencia
- Voxel grid es O(n), cluster extraction es O(n log n) — escalan bien
- Para point clouds grandes (>500K puntos), downsample agresivo primero (leaf 0.01)
- Procesar en C++ es ~10x más rápido que Python con PCL wrappers
- Usar `pcl_ros` para conversiones entre `sensor_msgs::PointCloud2` y `pcl::PointCloud`
- NUNCA procesar el point cloud completo de la cámara — SIEMPRE crop primero

### 2.4 Consideraciones especiales para ZED 2 stereo
- La ZED 2 es estéreo pasiva (no IR) → funciona bien con luz ambiente, pero mal en oscuridad total
- Superficies sin textura (paredes blancas, mesas lisas monocolor) producen depth pobre
  - Solución: asegurar que la iluminación sea suficiente y que haya textura visual en la escena
- Objetos transparentes/reflectivos: mejor que IR cameras pero aún problemáticos
  - Solución: usar RGB detection (YOLO/ZED Object Detection) y proyectar a 3D
- El FOV amplio (110°×70°) captura más escena que una RealSense, pero también más ruido en los bordes
  - Solución: crop más agresivo con PassThrough, especialmente en los bordes
- El depth neural mode da mejor calidad a corto alcance que el modo ULTRA
- Con eye-in-hand, la resolución efectiva del objeto mejora al acercarse (ventaja sobre cámara fija)
- **Auto-exposure puede causar flicker**: fijar exposure si la iluminación es controlada
- El `confidence_map` de la ZED es muy útil: filtrar puntos con confianza <50 mejora calidad del cloud

---

## 3. GPD (Grasp Pose Detection)

### 3.1 Qué es y cómo funciona
GPD toma un point cloud como input y genera candidatos de grasps 6-DOF para un parallel jaw gripper. Funciona en dos etapas:
1. **Sampling**: genera miles de candidatos de grasp en el point cloud
2. **Classification**: una CNN clasifica cada candidato como viable o no

El output es una lista de `GraspConfig` con posición, orientación, approach vector, y score.

### 3.2 Parámetros del gripper (DEBEN coincidir con el hardware real)
```yaml
# Estos parámetros definen el modelo del gripper para GPD
finger_width: 0.01        # Ancho de cada dedo (metros) — medir del gripper real
hand_outer_diameter: 0.085 # Distancia exterior entre dedos (apertura máxima)
hand_depth: 0.06           # Profundidad de agarre (cuánto penetra el dedo)
hand_height: 0.02          # Altura/grosor de los dedos
init_bite: 0.01            # Mínima penetración para considerar un grasp

# CUIDADO: si estos valores no coinciden con el gripper real,
# GPD generará grasps que colisionan o no alcanzan el objeto
```

### 3.3 Configuración de camera viewpoint (DINÁMICO con eye-in-hand)
```yaml
# Con eye-in-hand (ZED 2 en el gripper), el viewpoint CAMBIA con cada captura
# porque la cámara se mueve con el brazo.
#
# OPCIÓN A: Si el point cloud se transforma a base_link antes de enviarlo a GPD,
# el viewpoint debe ser la posición de la cámara EN base_link al momento de captura.
# Esto se calcula dinámicamente via TF (ver sección 2.1.1).
#
# OPCIÓN B: Si el point cloud se envía en el frame de la cámara,
# el viewpoint es (0, 0, 0) — correcto porque el origen es la cámara misma.

# Número de viewpoints
num_views: 1

# IMPORTANTE: NO dejar viewpoint hardcoded a (0,0,0) si el cloud está en base_link.
# Esto genera grasps con perspectiva incorrecta.
# Calcular dinámicamente en el código que llama a GPD.
```

**Implementación correcta para eye-in-hand:**
```python
def call_gpd(self, cluster_cloud):
    """Llama GPD con el viewpoint correcto basado en la posición actual de la ZED 2."""
    # Obtener posición actual de la cámara en base_link
    try:
        cam_tf = self.tf_buffer.lookup_transform(
            'base_link', 'zed2_left_camera_frame',
            rclpy.time.Time(), timeout=Duration(seconds=2.0))
        viewpoint = Point()
        viewpoint.x = cam_tf.transform.translation.x
        viewpoint.y = cam_tf.transform.translation.y
        viewpoint.z = cam_tf.transform.translation.z
    except Exception as e:
        self.get_logger().error(f"TF lookup failed, using last known viewpoint: {e}")
        viewpoint = self.last_known_viewpoint  # Fallback
    
    self.last_known_viewpoint = viewpoint
    
    # Construir request de GPD con viewpoint dinámico
    request = GenerateGrasps.Request()
    request.cloud = cluster_cloud
    request.view_point = viewpoint
    
    return self.gpd_client.call_async(request)
```

### 3.4 Workspace y sampling
```yaml
# Workspace limits (en el frame del point cloud)
# DEBE ser menor o igual al alcance del brazo
workspace:
  - -0.60   # x_min
  - 0.60    # x_max
  - -0.40   # y_min
  - 0.40    # y_max
  - 0.01    # z_min (justo arriba de la superficie)
  - 0.40    # z_max

# Número de samples (balance entre calidad y velocidad)
num_samples: 500         # 200-500 para competencia (velocidad)
                         # 1000+ para investigación (calidad)
num_threads: 4           # Ajustar al hardware disponible

# Filtros de calidad
min_inliers: 10          # Mínimos puntos dentro del grasp
```

### 3.5 Integración con ROS2
GPD se ejecuta típicamente como un service node:

```
/gpd_service (srv: GenerateGrasps)
  Input:  sensor_msgs/PointCloud2 (cluster del objeto)
  Output: GraspConfigList (lista de grasps ordenados por score)
```

Flujo:
1. Perception3D publica cluster del objeto target
2. Se llama al servicio de GPD con el cluster
3. GPD retorna N grasps ordenados por score
4. El PickManager itera sobre los grasps intentando cada uno

### 3.6 Problemas comunes y soluciones

**Problema: GPD no genera grasps**
- Causa: point cloud muy sparse o fuera del workspace
- Solución: verificar que el cluster tiene suficientes puntos (>100), ajustar workspace

**Problema: Grasps generados son inalcanzables por el brazo**
- Causa: GPD no conoce la cinemática del brazo
- Solución: post-filtrar con IK check antes de intentar planear

**Problema: Grasps colisionan con la mesa**
- Causa: z_min del workspace incluye la mesa, o hand_depth muy grande
- Solución: ajustar z_min a table_height + 0.02, ajustar hand_depth

**Problema: Grasps para objetos planos (cubiertos, tarjetas) fallan**
- Causa: GPD necesita volumen 3D para generar grasps; objetos planos no dan suficiente
- Solución: usar un FlatGraspEstimator que genere grasps top-down basados en el centroide y orientación del cluster

---

## 4. MOTION PLANNING (MoveIt2)

### 4.1 pymoveit2 (Python wrapper)
```python
from pymoveit2 import MoveIt2

moveit2 = MoveIt2(
    node=self,
    joint_names=[f"joint{i}" for i in range(1, 7)],
    base_link_name="link_base",
    end_effector_name="link_eef",
    group_name="xarm6",
    callback_group=ReentrantCallbackGroup(),
)

# Velocidad y aceleración (fracción de los máximos)
moveit2.max_velocity = 0.3      # 30% de velocidad máxima
moveit2.max_acceleration = 0.3

# Movimiento a pose cartesiana
moveit2.move_to_pose(
    position=[x, y, z],          # metros, en base_link
    quat_xyzw=[qx, qy, qz, qw], # orientación como quaternion
    cartesian=False,              # True para línea recta
    tolerance_position=0.001,     # 1mm de tolerancia
    tolerance_orientation=0.01,   # ~0.6° de tolerancia
)
result = moveit2.wait_until_executed()

# Movimiento a configuración de joints
moveit2.move_to_configuration(
    [j1, j2, j3, j4, j5, j6]  # radianes
)
result = moveit2.wait_until_executed()

# Movimiento cartesiano (línea recta) — para approach y retract
waypoints = [current_pose, pre_grasp_pose, grasp_pose]
moveit2.move_to_pose(
    position=grasp_position,
    quat_xyzw=grasp_orientation,
    cartesian=True,   # IMPORTANTE: True para descenso lineal al grasp
)
```

### 4.2 Planners disponibles
| Planner | Tipo | Velocidad | Calidad path | Uso recomendado |
|---------|------|-----------|--------------|-----------------|
| OMPL RRTConnect | Sampling | Rápido | Variable | Default, movimientos libres |
| OMPL RRT* | Sampling | Lento | Óptimo | Espacios complejos |
| Pilz PTP | Determinístico | Muy rápido | Point-to-point | Movimientos simples |
| Pilz LIN | Determinístico | Muy rápido | Línea recta | Approach/retract |
| STOMP | Optimization | Medio | Smooth | Paths suaves |
| CHOMP | Optimization | Medio | Obstacle-aware | Entornos clutterados |

Para competencia: **OMPL RRTConnect** como default + **Pilz LIN** para approach/retract.

### 4.3 Collision Management
```python
# Agregar mesa como collision object
moveit2.add_collision_box(
    id="table",
    position=[0.5, 0.0, table_height / 2],  # centro del box
    quat_xyzw=[0, 0, 0, 1],
    size=[1.0, 0.8, table_height],  # largo, ancho, alto
    frame_id="base_link",
)

# Agregar estante/shelf
moveit2.add_collision_box(
    id="shelf_back",
    position=[shelf_x, 0.0, shelf_height / 2],
    quat_xyzw=[0, 0, 0, 1],
    size=[0.02, 1.0, shelf_height],
)

# Attach objeto agarrado al end-effector (evita self-collision)
moveit2.attach_collision_object(
    id="grasped_object",
    link="link_eef",
    touch_links=["link_eef", "link6", "left_finger", "right_finger"],
)

# Detach después de soltar
moveit2.detach_collision_object(id="grasped_object")
moveit2.remove_collision_object(id="grasped_object")
```

REGLA: `touch_links` DEBE incluir TODOS los links del gripper. Si falta alguno, MoveIt2 rechazará grasps válidos por "colisión" del gripper con el objeto.

### 4.4 Planning scene management
- SIEMPRE agregar collision objects de superficies ANTES de planear
- ACTUALIZAR la scene cuando el robot o los objetos se mueven
- REMOVER objetos de la scene después de agarrarlos (y hacer attach)
- Los collision objects persisten entre llamadas — no re-agregar cada vez
- Usar `moveit2.clear_all_collision_objects()` al inicio de cada task para empezar limpio

---

## 5. PIPELINE DE PICK AND PLACE COMPLETA

### 5.1 Flujo de alto nivel
```
[A] Preparación
    1. Mover brazo a pose de observación (stare_at_table)
    2. Esperar estabilización de cámara (~0.5s)
    3. Agregar collision objects de superficies conocidas

[B] Percepción
    4. Capturar point cloud
    5. Ejecutar Perception3D pipeline (voxel → crop → RANSAC → cluster)
    6. Identificar objeto target (por posición, color, o clasificación)
    7. Extraer cluster del objeto target

[C] Grasp Generation
    8. Enviar cluster a GPD (o FlatGraspEstimator para objetos planos)
    9. Recibir lista de grasps candidatos
    10. Filtrar grasps inalcanzables (IK feasibility check)
    11. Ordenar por score (GPD score × reachability score)

[D] Ejecución del Pick
    Para cada grasp candidato (mejor primero):
        12. Calcular pre-grasp pose (5-10cm arriba del grasp, misma orientación)
        13. Abrir gripper
        14. Planear y ejecutar movimiento libre a pre-grasp
        15. Si plan falla → siguiente candidato
        16. Planear y ejecutar movimiento cartesiano (línea recta) de pre-grasp → grasp
        17. Si plan falla → retornar a pose segura, siguiente candidato
        18. Cerrar gripper
        19. VERIFICAR GRASP (ver sección 5.2)
        20. Si verificación falla → abrir gripper, retornar a pre-grasp, siguiente candidato
        21. Planear y ejecutar retract (línea recta, 10cm arriba)
        22. Attach collision object del objeto al gripper
        23. Break — pick exitoso

[E] Transport
    24. Planear movimiento libre a zona de destino (con objeto attached)
    25. Ejecutar movimiento

[F] Place
    26. Calcular pose de place (encima del destino)
    27. Planear y ejecutar descenso cartesiano
    28. Abrir gripper
    29. Detach collision object
    30. Ejecutar retract (línea recta, arriba)
    31. Mover brazo a pose segura/home

[G] Recovery (si todo falla)
    32. Si todos los grasps fallan → RE-PERCEPCIÓN (volver a paso 4)
    33. Si re-percepción falla 2 veces → reportar fallo, pedir DEM, o skip
```

### 5.2 Verificación de grasp
Después de cerrar el gripper, SIEMPRE verificar que se agarró algo:

**Método 1: Posición del gripper (más simple y confiable)**
```python
# Leer posición actual del gripper
gripper_pos = get_gripper_position()  # 0.0 = cerrado, 0.85 = abierto

FULLY_CLOSED = 0.02   # Si está casi cerrado, no agarró nada
MIN_OBJECT = 0.03     # Mínimo para que haya algo entre los dedos

if gripper_pos < FULLY_CLOSED:
    # Gripper cerró completamente → no hay objeto
    return False
else:
    # Gripper se detuvo → hay algo entre los dedos
    return True
```

**Método 2: Verificación visual post-grasp**
```python
# Después de cerrar y subir, re-capturar image
# Si el objeto ya no está donde estaba → pick exitoso
# Si sigue ahí → pick falló
```

**Método 3: Fuerza/torque (si hay sensor F/T)**
```python
# Monitorear fuerza durante el descenso
# Si fuerza Z excede threshold → contacto con objeto o superficie
# Útil para force-guarded descent
```

### 5.3 Flat Grasp Estimator (para objetos planos)
Para cubiertos, tarjetas, objetos delgados donde GPD no funciona:

```python
def estimate_flat_grasp(cluster, table_normal):
    """
    Genera grasp top-down basado en PCA del cluster.
    Approach: vertical desde arriba.
    Orientación: alineado con el eje mayor del objeto.
    """
    # Centroide del cluster
    centroid = compute_centroid(cluster)

    # PCA para obtener orientación principal
    eigenvalues, eigenvectors = compute_pca(cluster)
    major_axis = eigenvectors[:, 0]  # Eje más largo del objeto

    # Grasp position: centroide del objeto, a la altura de la mesa
    grasp_pos = [centroid.x, centroid.y, centroid.z]

    # Grasp orientation: approach vertical, aligned con major axis
    # Z del gripper apunta hacia abajo (approach direction)
    # X del gripper alineado con el eje menor (los dedos a lo largo del eje menor)
    grasp_quat = compute_top_down_quaternion(major_axis, table_normal)

    return GraspPose(position=grasp_pos, orientation=grasp_quat, score=0.8)
```

### 5.4 Timing en competencia
Budget de tiempo para pick-and-place en competencia (7 min total para PPC):

| Fase | Tiempo máximo | Notas |
|------|:------------:|-------|
| Navegar a mesa | 15-30s | Depende de distancia |
| Percepción | 2-5s | Pipeline PCL + clasificación |
| GPD | 3-8s | Depende de num_samples |
| Planning (por intento) | 1-3s | OMPL RRTConnect |
| Ejecución de pick | 5-10s | Incluye approach + grasp + retract |
| Verificación de grasp | 0.5s | Check gripper position |
| Transporte | 10-20s | Nav + arm movement |
| Place | 5-10s | Approach + release + retract |
| **Total por objeto** | **40-90s** | **~4-8 objetos en 7 min** |

Si un grasp falla, el retry toma ~10-15s extra. Con 3 retries max por objeto, abortar y pasar al siguiente si excede 120s total.

---

## 6. PATRONES DE ROBUSTEZ PARA COMPETENCIA

### 6.1 Watchdog timer por operación
```python
import asyncio

async def pick_with_timeout(target, timeout=90.0):
    try:
        result = await asyncio.wait_for(
            pick_object(target),
            timeout=timeout
        )
        return result
    except asyncio.TimeoutError:
        logger.warn(f"Pick timeout after {timeout}s, aborting")
        move_to_safe_pose()
        return False
```

### 6.2 Service call con timeout
```python
def call_service_safe(client, request, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        logger.error(f"Service {client.srv_name} not available")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.done():
        return future.result()
    else:
        logger.error(f"Service call timed out")
        return None
```

### 6.3 Re-percepción en caso de fallo
```python
MAX_PERCEPTION_RETRIES = 3

for attempt in range(MAX_PERCEPTION_RETRIES):
    clusters = perceive_objects()
    grasps = generate_grasps(target_cluster)

    if try_all_grasps(grasps):
        return True  # Pick exitoso

    # Todos los grasps fallaron — re-percibir
    logger.info(f"All grasps failed, re-perceiving (attempt {attempt+1})")
    move_to_different_viewpoint()  # Ligeramente diferente para nueva perspectiva
    time.sleep(0.5)  # Esperar estabilización

return False  # Fallo definitivo
```

### 6.4 Manejo de errores del xArm
```python
def handle_xarm_error():
    """Recuperar de error del controlador xArm."""
    err = get_error_code()
    if err == 0:
        return True  # Sin error

    logger.warn(f"xArm error code: {err}")

    # Limpiar error y re-habilitar
    clean_error()
    motion_enable(id=8, data=1)
    set_mode(data=0)
    set_state(data=0)
    time.sleep(0.5)

    # Verificar que se limpió
    err = get_error_code()
    if err != 0:
        logger.error(f"Cannot clear xArm error: {err}")
        return False
    return True
```

---

## 7. REGLAS Y MEJORES PRÁCTICAS

### Código
1. TODOS los valores numéricos que dependen del hardware → parámetros ROS2
2. NUNCA `time.sleep()` en callbacks — usar timers o `asyncio`
3. SIEMPRE `try/except` en callbacks de subscribers y services
4. Usar `self.get_logger()` — NUNCA `print()`
5. Service calls SIEMPRE con timeout
6. Futures SIEMPRE verificar `.done()` antes de usar `.result()`

### Percepción
7. SIEMPRE crop el point cloud antes de procesar (PassThrough)
8. SIEMPRE esperar ~0.5s después de mover la cámara antes de capturar
9. Los parámetros de clustering dependen del tipo de objetos — tunear por task
10. Publicar point clouds intermedios para debug en RViz

### Grasping
11. SIEMPRE verificar grasp post-cierre de gripper
12. Implementar re-percepción si todos los grasps fallan
13. Para objetos planos, usar FlatGraspEstimator en vez de GPD
14. Pre-grasp → Grasp SIEMPRE en movimiento cartesiano (línea recta)
15. Retract SIEMPRE en movimiento cartesiano (línea recta hacia arriba)
16. Máximo 3 intentos de grasp por objeto antes de abortar

### MoveIt2
17. SIEMPRE agregar collision objects de superficies antes de planear
18. `touch_links` debe incluir TODOS los links del gripper
19. Attach objeto al gripper después de agarrar, detach después de soltar
20. Limpiar planning scene al inicio de cada task

### Competencia
21. Budget de tiempo por objeto: máximo 120s, abort si excede
22. Priorizar objetos fáciles primero (más grandes, mejor posicionados)
23. Skip objetos en posiciones complicadas (borde de mesa, detrás de otros)
24. Si el brazo entra en error, SIEMPRE limpiar error + re-enable antes de continuar
25. Tener poses de "safe home" predefinidas para recovery rápido
