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

        self.get_logger().info('VAMP server for FRIDA is ready and waiting for planning requests')

    def plan_callback(self, request, response):
        self.get_logger().info("Received new planning request")
        
        try:
            env = vamp.Environment()
            centers = np.array(request.sphere_centers_flat, dtype=np.float64)
            radii = np.array(request.sphere_radii, dtype=np.float64)

            self.get_logger().info("Processing collision environment:")
            for i in range(len(radii)):
                idx = i * 3
                pos = centers[idx:idx+3]
                self.get_logger().info(f"  Sphere {i+1}: center=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}], radius={radii[i]:.4f}")
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
                
                step = 0.05 
                nx = max(2, int(lx / step))
                ny = max(2, int(ly / step))
                nz = max(2, int(lz / step))
                
                sphere_radius = 0.03

                self.get_logger().info(f"  Box {i+1}: size={lx:.3f}x{ly:.3f}x{lz:.3f}, approximating with {nx*ny*nz} spheres")
                
                for dx in np.linspace(-lx/2.0, lx/2.0, nx):
                    for dy in np.linspace(-ly/2.0, ly/2.0, ny):
                        for dz in np.linspace(-lz/2.0, lz/2.0, nz):
                            sphere_center = [float(pos[0] + dx), float(pos[1] + dy), float(pos[2] + dz)]
                            env.add_sphere(vamp.Sphere(sphere_center, float(sphere_radius)))
            finger_left = 0.8
            finger_right = 0.8
            start_8 = np.array(list(request.start_state) + [finger_left, finger_right], dtype=np.float64)
            goal_8 = np.array(list(request.goal_state) + [finger_left, finger_right], dtype=np.float64)

            is_start_valid = vamp.frida_real.validate(start_8, env)
            is_goal_valid = vamp.frida_real.validate(goal_8, env)

            if not is_start_valid:
                self.get_logger().error("Start state is in collision with environment or self-colliding")
            if not is_goal_valid:
                self.get_logger().error("Goal state is in collision with environment or self-colliding")

            if not is_start_valid or not is_goal_valid:
                clean_env = vamp.Environment()
                if vamp.frida_real.validate(start_8, clean_env) and vamp.frida_real.validate(goal_8, clean_env):
                     self.get_logger().info("Note: Both states are valid without obstacles. The collision is likely due to self-collision or joint limits")
                else:
                     self.get_logger().error("States are invalid even without obstacles. Check joint values and ensure they're within limits and not self-colliding")
                
                response.success = False
                return response

            self.get_logger().info("Start and goal states validated successfully. Starting RRT-Connect planner...")

            
            rng = vamp.frida_real.xorshift()
            result = vamp.frida_real.rrtc(start_8, goal_8, env, self.settings, rng)
            
            if result and result.solved:
                raw_path = result.path
                n_points = len(raw_path)
                
                self.get_logger().info(f"Planning succeeded with {n_points} waypoints. Now validating and densifying path...")
                
                path_nodes = []
                for i in range(n_points):
                    idx = int(i)
                    wp_tensor = raw_path[idx]
                    wp_array = np.array(list(wp_tensor), dtype=np.float64)
                    path_nodes.append(wp_array)
                
                self.get_logger().info("Running collision validation on interpolated path segments...")
                
                dense_path = []
                path_is_safe = True
                
                for i in range(n_points - 1):
                    p1 = path_nodes[i]
                    p2 = path_nodes[i+1]
                    
                    
                    node_distance = np.linalg.norm(p1[:6] - p2[:6])
                    num_steps = max(2, int(node_distance / 0.04))
                    
                    for t in np.linspace(0, 1, num_steps, endpoint=False):
                        interp_wp = p1 + (p2 - p1) * t
                        
                        if not vamp.frida_real.validate(interp_wp, env):
                            self.get_logger().error(f"Collision detected during path interpolation at segment {i}")
                            path_is_safe = False
                            break
                            
                        
                        dense_path.append(interp_wp)
                        
                    if not path_is_safe:
                        break
                        
                if path_is_safe:
                    
                    dense_path.append(path_nodes[-1])
                    
                    
                    flat_path = []
                    clean_points = [dense_path[0]]
                    
                    
                    for wp in dense_path[1:]:
                        dist = np.linalg.norm(wp[:6] - clean_points[-1][:6])
                        if dist >= 0.02: 
                            clean_points.append(wp)
                            
                    
                    final_distance = np.linalg.norm(dense_path[-1][:6] - clean_points[-1][:6])
                    if final_distance > 0.001:
                        clean_points.append(dense_path[-1])
                        
                    
                    for wp in clean_points:
                        for j in range(6):
                            flat_path.append(float(wp[j]))
                            
                    response.waypoints_flat = flat_path
                    response.success = True
                    self.get_logger().info(f"Path generation complete: refined from {n_points} raw waypoints to {len(clean_points)} smooth waypoints")
                else:
                    self.get_logger().warn("Path validation failed due to internal collision. Please try planning again")
                    response.success = False
            else:
                self.get_logger().warn("VAMP planner could not find a valid path within the given constraints")
                response.success = False
                
        except Exception as e:
            self.get_logger().error(f"Planning failed with exception: {e}")
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