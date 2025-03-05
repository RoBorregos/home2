#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import MoveToPose, MoveJoints
from frida_interfaces.srv import GetJoints, AddCollisionObject
from frida_motion_planning.utils.MoveItPlanner import MoveItPlanner


class MotionPlanningServer(Node):
    def __init__(self):
        super().__init__("pick_server")
        self.callback_group = ReentrantCallbackGroup()

        self._move_to_pose_server = ActionServer(
            self,
            MoveToPose,
            "/manipulation/move_to_pose_action_server",
            self.move_to_pose_execute_callback,
            callback_group=self.callback_group,
        )

        self._move_joints_server = ActionServer(
            self,
            MoveJoints,
            "/manipulation/move_joints_action_server",
            self.move_joints_execute_callback,
            callback_group=self.callback_group,
        )

        self.get_joints_service = self.create_service(
            GetJoints, "/manipulation/get_joints", self.get_joints_callback
        )

        self.add_collision_object_service = self.create_service(
            AddCollisionObject,
            "/manipulation/add_collision_object",
            self.add_collision_object_callback,
        )

        # Here we can select other planner (if implemented)
        self.planner = MoveItPlanner(self, self.callback_group)
        self.planner.set_velocity(0.15)
        self.planner.set_acceleration(0.15)
        self.planner.set_planner("RRTConnect")
        self.get_logger().info("Pick Action Server has been started")

    async def move_to_pose_execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing pick goal...")

        # Initialize result
        feedback = MoveToPose.Feedback()
        result = MoveToPose.Result()
        self.set_planning_settings(goal_handle)
        try:
            result.success = self.move_to_pose(goal_handle, feedback)
            self.get_logger().info(
                "Move to pose finished with result: " + str(result.success)
            )
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Move to pose failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    async def move_joints_execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing pick goal...")

        # Initialize result
        feedback = MoveJoints.Feedback()
        result = MoveJoints.Result()
        self.set_planning_settings(goal_handle)
        try:
            result.success = self.move_joints(goal_handle, feedback)
            self.get_logger().info(
                "Move joints finished with result: " + str(result.success)
            )
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Move joints failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    def move_to_pose(self, goal_handle, feedback):
        """Perform the pick operation."""
        self.get_logger().info(f"Moving to pose: {goal_handle.request.pose}")
        pose = goal_handle.request.pose
        result = self.planner.plan_pose_goal(
            pose,
            wait=True,
        )
        return result

    def move_joints(self, goal_handle, feedback):
        self.get_logger().info(
            f"Moving joints: {goal_handle.request.joint_names} to joints: {list(goal_handle.request.joint_positions)}"
        )
        joint_names = goal_handle.request.joint_names
        joint_positions = list(goal_handle.request.joint_positions)
        result = self.planner.plan_joint_goal(
            joint_positions,
            joint_names,
            wait=True,
        )
        return result

    def set_planning_settings(self, goal_handle):
        velocity = (
            goal_handle.request.velocity if goal_handle.request.velocity else 0.15
        )
        acceleration = (
            goal_handle.request.acceleration
            if goal_handle.request.acceleration
            else 0.15
        )
        planner_id = (
            goal_handle.request.planner_id
            if len(goal_handle.request.planner_id) != 0
            else "RRTConnect"
        )
        self.planner.set_velocity(velocity)
        self.planner.set_acceleration(acceleration)
        self.planner.set_planner(planner_id)

    def get_joints_callback(self, request, response):
        joint_dict = self.planner.get_joint_positions()
        for joint_name in joint_dict:
            response.joint_positions.append(float(joint_dict[joint_name]))
            response.joint_names.append(joint_name)
        return response

    """
    string id
    string type
    geometry_msgs/PoseStamped pose
    geometry_msgs/Point dimensions # length, width, height
    shape_msgs/Mesh
    string path_to_mesh
    ---
    bool success
    """

    def add_collision_object_callback(self, request, response):
        """Handle requests to add collision objects to the planning scene"""
        try:
            # Generate a unique ID for the collision object
            object_id = f"{request.id}"

            # Handle different collision object types
            if request.type == "box":
                self.planner.add_collision_box(
                    id=object_id,
                    size=(
                        request.dimensions.x,
                        request.dimensions.y,
                        request.dimensions.z,
                    ),
                    pose=request.pose,
                )
                self.get_logger().info(f"Added collision box: {object_id}")

            elif request.type == "sphere":
                # For spheres, use the x component of dimensions as radius
                self.planner.add_collision_sphere(
                    id=object_id, radius=request.dimensions.x, pose=request.pose
                )
                self.get_logger().info(f"Added collision sphere: {object_id}")

            elif request.type == "cylinder":
                # For cylinders, use x as radius, z as height
                self.planner.add_collision_cylinder(
                    id=object_id,
                    height=request.dimensions.z,
                    radius=request.dimensions.x,
                    pose=request.pose,
                )
                self.get_logger().info(f"Added collision cylinder: {object_id}")

            elif request.type == "mesh":
                # Add collision mesh from file path -> Priority to file path
                if request.path_to_mesh:
                    self.planner.add_collision_mesh(
                        id=object_id,
                        filepath=request.path_to_mesh,
                        pose=request.pose,
                        scale=(
                            request.dimensions.x if request.dimensions.x != 0.0 else 1.0
                        ),
                    )
                    self.get_logger().info(
                        f"Added collision mesh from file: {object_id}"
                    )
                # Or from mesh data
                else:
                    import trimesh

                    # Convert mesh data to trimesh object
                    mesh = trimesh.Trimesh()
                    mesh.vertices = [(v.x, v.y, v.z) for v in request.mesh.vertices]
                    mesh.faces = [
                        (t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2])
                        for t in request.mesh.triangles
                    ]

                    # transform from
                    self.planner.add_collision_mesh(
                        id=object_id,
                        filepath=None,
                        pose=request.pose,
                        mesh=mesh,
                        frame_id=request.pose.header.frame_id,
                    )
                    self.get_logger().info(
                        f"Added collision mesh from data: {object_id}"
                    )
            else:
                self.get_logger().error(
                    f"Unsupported collision object type: {request.type}"
                )
                response.success = False
                return response

            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to add collision object: {str(e)}")
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = MotionPlanningServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
