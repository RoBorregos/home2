
import sys
import rclpy
from rclpy.node import Node
from vamp_moveit_plugin.srv import VampPlan
import numpy as np
import math

sys.path.append('/home/dominguez/roborregos/home_ws/src/manipulation/packages/vamp/src')
import vamp

class VampServer(Node):
    def __init__(self):
        super().__init__('vamp_server')
        self.srv = self.create_service(VampPlan, 'plan_vamp_path', self.plan_callback)
        
        self.settings = vamp.RRTCSettings()
        self.settings.max_iterations = 20000
        self.settings.range = 0.05

        self.get_logger().info('Server FRIDA ready. Waiting for planning of all joints...')

    def plan_callback(self, request, response):
        self.get_logger().info("--- Planning Attempt ---")
        
        try:
            
            env = vamp.Environment()
            centers = np.array(request.sphere_centers_flat, dtype=np.float64)
            radii = np.array(request.sphere_radii, dtype=np.float64)


            self.get_logger().info("Inspection Obstacles Received:")
            for i in range(len(radii)):
                idx = i * 3
                pos = centers[idx:idx+3]
                self.get_logger().info(f"   -> Sphere {i+1}: Position [X: {pos[0]:.4f}, Y: {pos[1]:.4f}, Z: {pos[2]:.4f}] | Radius: {radii[i]:.4f}")
                env.add_sphere(vamp.Sphere(pos, radii[i]))

            box_centers = np.array(request.box_centers_flat, dtype=np.float64)
            box_sizes = np.array(request.box_sizes_flat, dtype=np.float64)
            
            num_boxes = len(box_sizes) // 3
            for i in range(num_boxes):
                idx = i * 3
                
                pos = [float(x) for x in box_centers[idx:idx+3]]
                
                
                lx = float(box_sizes[idx])
                ly = float(box_sizes[idx+1])
                lz = float(box_sizes[idx+2])
                
                
                paso = 0.05 
                nx = max(2, int(lx / paso))
                ny = max(2, int(ly / paso))
                nz = max(2, int(lz / paso))
                
                
                radio_esfera = 0.03

                self.get_logger().info(f"   ->  Box {i+1} of {lx}x{ly}x{lz} with {nx*ny*nz} exact spheres...")
                
                for dx in np.linspace(-lx/2.0, lx/2.0, nx):
                    for dy in np.linspace(-ly/2.0, ly/2.0, ny):
                        for dz in np.linspace(-lz/2.0, lz/2.0, nz):
                            centro_esfera = [float(pos[0] + dx), float(pos[1] + dy), float(pos[2] + dz)]
                            env.add_sphere(vamp.Sphere(centro_esfera, float(radio_esfera)))
                    

            finger_left = 0.8
            finger_right = 0.8
            start_8 = np.array(list(request.start_state) + [finger_left, finger_right], dtype=np.float64)
            goal_8 = np.array(list(request.goal_state) + [finger_left, finger_right], dtype=np.float64)

            is_start_valid = vamp.frida_real.validate(start_8, env)
            is_goal_valid = vamp.frida_real.validate(goal_8, env)

            if not is_start_valid:
                self.get_logger().error("VAMP reject the START: The white arm is already touching the sphere (or itself).")
            if not is_goal_valid:
                self.get_logger().error("VAMP reject the GOAL: The orange arm is already touching the sphere (or itself).")

            if not is_start_valid or not is_goal_valid:
                
                env_limpio = vamp.Environment()
                if vamp.frida_real.validate(start_8, env_limpio) and vamp.frida_real.validate(goal_8, env_limpio):
                     self.get_logger().info("Info: Without obstacles, both START and GOAL are valid. The issue is likely due to the arm colliding with itself or its base configuration (Auto-collision).")
                else:
                     self.get_logger().error("Error: Even without obstacles, either START or GOAL is invalid. Please check the joint values and ensure they are within the robot's limits and not in a self-colliding configuration.")
                
                response.success = False
                return response

            self.get_logger().info("START and GOAL are clean. Launching RRTC...")

            
            rng = vamp.frida_real.xorshift()
            result = vamp.frida_real.rrtc(start_8, goal_8, env, self.settings, rng)
            
            if result and result.solved:
                raw_path = result.path
                n_points = len(raw_path)
                
                self.get_logger().info(f"✨ RRT Solved con {n_points} nodos. Aislamiento de tensores...")
                
                
                path_nodos = []
                for i in range(n_points):
                    
                    idx = int(i)
                    
                    wp_tensor = raw_path[idx]
                    wp_array = np.array(list(wp_tensor), dtype=np.float64)
                    path_nodos.append(wp_array)
                
                self.get_logger().info("🔍 Iniciando Microscopio de VAMP (Validación de aristas)...")
                
                
                flat_path = []
                ruta_es_segura = True
                
                for i in range(n_points - 1):
                    p1 = path_nodos[i]
                    p2 = path_nodos[i+1]
                    
                    
                    pasos = 10
                    for t in np.linspace(0, 1, pasos, endpoint=False):
                        interp_wp = p1 + (p2 - p1) * t
                        
                        
                        if not vamp.frida_real.validate(interp_wp, env):
                            self.get_logger().error(f"❌ VAMP se atrapó a sí mismo chocando entre el nodo {i} y {i+1}. Ruta rechazada.")
                            ruta_es_segura = False
                            break
                        
                        
                        for j in range(6):
                            flat_path.append(float(interp_wp[j]))
                            
                    if not ruta_es_segura:
                        break
                
                if ruta_es_segura:
                    
                    for j in range(6):
                        flat_path.append(float(path_nodos[-1][j]))
                    
                    response.waypoints_flat = flat_path
                    response.success = True
                    self.get_logger().info(f"✅ Validación perfecta. Enviando {len(flat_path)//6} waypoints a MoveIt.")
                else:
                    self.get_logger().warn("⚠️ RRT cortó una esquina peligrosa. Mueve ligeramente el brazo en RViz e intenta de nuevo.")
                    response.success = False
            else:
                self.get_logger().warn("⚠️ VAMP no pudo encontrar una ruta válida.")
                response.success = False
                
        except Exception as e:
            self.get_logger().error(f"Error in the server: {e}")
            response.success = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VampServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()