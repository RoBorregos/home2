
import sys
import rclpy
from rclpy.node import Node
from vamp_moveit_plugin.srv import VampPlan
import numpy as np


sys.path.append('/home/dominguez/roborregos/home_ws/src/manipulation/packages/vamp/src')
import vamp

class VampServer(Node):
    def __init__(self):
        super().__init__('vamp_server')
        self.srv = self.create_service(VampPlan, 'plan_vamp_path', self.plan_callback)
        
        self.settings = vamp.RRTCSettings()
        self.settings.max_iterations = 2000

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

            
            
            dedo_izq = 0.8
            dedo_der = 0.8
            start_8 = np.array(list(request.start_state) + [dedo_izq, dedo_der], dtype=np.float64)
            goal_8 = np.array(list(request.goal_state) + [dedo_izq, dedo_der], dtype=np.float64)

            
            
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
                self.get_logger().info(f"RRT Solved: {n_points} points found.")
                
                
                
                flat_path = []
                
                
                
                for i in range(n_points - 1):
                    p1 = np.array(raw_path[i])
                    p2 = np.array(raw_path[i+1])
                    
                    
                    for t in np.linspace(0, 1, 10, endpoint=False):
                        interp_wp = p1 + (p2 - p1) * t
                        
                        for j in range(6):
                            flat_path.append(float(interp_wp[j]))
                
                
                last_wp = raw_path[n_points-1]
                for j in range(6):
                    flat_path.append(float(last_wp[j]))
                
                response.waypoints_flat = flat_path
                response.success = True
                self.get_logger().info(f"Sending {len(flat_path)//6} waypoints to MoveIt.")
            else:
                self.get_logger().warn("VAMP could not find a valid path.")
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