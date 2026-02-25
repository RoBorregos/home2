
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
        
        self.get_logger().info('Frida RRTC VAMP Server ready to receive planning requests.')

    def plan_callback(self, request, response):
        self.get_logger().info("--- Planning Attempt ---")
        
        try:
            
            env = vamp.Environment()
            
            
            centers = np.array(request.sphere_centers_flat, dtype=np.float64)
            radii = np.array(request.sphere_radii, dtype=np.float64)
            for i in range(len(radii)):
                idx = i * 3
                env.add_sphere(vamp.Sphere(centers[idx:idx+3], radii[i]))

            
            
            start_8 = np.array(list(request.start_state) + [0.0, 0.0], dtype=np.float64)
            goal_8 = np.array(list(request.goal_state) + [0.0, 0.0], dtype=np.float64)

            
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
                    
                    
                    for t in np.linspace(0, 1, 100, endpoint=False):
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
            self.get_logger().error(f"Error in server: {e}")
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