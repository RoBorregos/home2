#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_srvs.srv import SetBool
from frida_interfaces.action import MoveToPose, MoveJoints
from frida_constants.manipulation_constants import (
    GET_COLLISION_OBJECTS_SERVICE,
)
from frida_interfaces.srv import (
    GetJoints,
    AddCollisionObjects,
    RemoveCollisionObject,
    ToggleServo,
    AttachCollisionObject,
    GetCollisionObjects,
)
from frida_constants.manipulation_constants import (
    ALWAYS_SET_MODE,
    MOVEIT_MODE,
    JOINT_VELOCITY_MODE,
    SET_JOINT_VELOCITY_SERVICE,
    PICK_PLANNER,
    MOVE_TO_POSE_ACTION_SERVER,
    DEBUG_POSE_GOAL_TOPIC,
    MOVE_JOINTS_ACTION_SERVER,
    GET_JOINT_SERVICE,
    ADD_COLLISION_OBJECT_SERVICE,
    REMOVE_COLLISION_OBJECT_SERVICE,
    ATTACH_COLLISION_OBJECT_SERVICE,
    TOGGLE_SERVO_SERVICE,
    GRIPPER_SET_STATE_SERVICE,
    MIN_CONFIGURATION_DISTANCE_TRESHOLD,
)
from xarm_msgs.srv import MoveVelocity
from frida_interfaces.msg import CollisionObject
from frida_motion_planning.utils.MoveItPlanner import MoveItPlanner
from frida_motion_planning.utils.MoveItServo import MoveItServo
from frida_motion_planning.utils.XArmServices import XArmServices


class MotionPlanningServer(Node):
    def __init__(self):
        super().__init__("motion_planning_server")
        self.callback_group = ReentrantCallbackGroup()

        # Here we can select other planner (if implemented)
        self.planner = MoveItPlanner(self, self.callback_group)
        self.planner.set_velocity(0.15)
        self.planner.set_acceleration(0.15)
        self.planner.set_planning_time(0.5)
        self.planner.set_planner(PICK_PLANNER)

        self.servo = MoveItServo(
            self,
            self.callback_group,
            max_velocity=0.1,
            max_acceleration=0.1,
        )

        self._move_to_pose_server = ActionServer(
            self,
            MoveToPose,
            MOVE_TO_POSE_ACTION_SERVER,
            self.move_to_pose_execute_callback,
            callback_group=self.callback_group,
        )

        self._debug_pose_publisher = self.create_publisher(
            PoseStamped, DEBUG_POSE_GOAL_TOPIC, 10
        )

        self._move_joints_server = ActionServer(
            self,
            MoveJoints,
            MOVE_JOINTS_ACTION_SERVER,
            self.move_joints_execute_callback,
            callback_group=self.callback_group,
        )

        self.get_joints_service = self.create_service(
            GetJoints, GET_JOINT_SERVICE, self.get_joints_callback
        )

        self.add_collision_object_service = self.create_service(
            AddCollisionObjects,
            ADD_COLLISION_OBJECT_SERVICE,
            self.add_collision_objects_callback,
        )

        self.remove_collision_object_service = self.create_service(
            RemoveCollisionObject,
            REMOVE_COLLISION_OBJECT_SERVICE,
            self.remove_collision_object_callback,
        )

        self.attach_collision_object_service = self.create_service(
            AttachCollisionObject,
            ATTACH_COLLISION_OBJECT_SERVICE,
            self.attach_collision_object_callback,
        )

        self.gripper_set_state_service = self.create_service(
            SetBool,
            GRIPPER_SET_STATE_SERVICE,
            self.set_gripper_state_callback,
        )

        self.toggle_servo_service = self.create_service(
            ToggleServo, TOGGLE_SERVO_SERVICE, self.toggle_servo_callback
        )

        self.servo_speed_subscriber = self.create_subscription(
            TwistStamped,
            "/manipulation/servo_speed",
            self.servo_speed_callback,
            10,
            callback_group=self.callback_group,
        )

        # is MoveItPlanner could not spawn services, send None
        # TODO: I changed my mind, set_mode goes in this script, not on the MoveItPlanner
        if self.planner.mode_enabled:
            self.xarm_services = XArmServices(
                self, self.planner.mode_client, self.planner.state_client
            )
            self.real_xarm = True
        else:
            self.xarm_services = XArmServices(self, None, None)
            self.real_xarm = False

        self.xarm_joint_velocity_service = self.create_service(
            MoveVelocity,
            SET_JOINT_VELOCITY_SERVICE,
            self.set_joint_velocity_callback,
        )

        self.current_mode = -1

        self.get_logger().info("Motion Planning Server has been started")
        self.get_collision_objects_service = self.create_service(
            GetCollisionObjects,
            GET_COLLISION_OBJECTS_SERVICE,
            self.get_collision_objects_callback,
        )

        self.get_logger().info("Motion Planning Action Server has been started")

    def move_to_pose_execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing pose goal...")

        # Initialize result
        self._debug_pose_publisher.publish(goal_handle.request.pose)
        feedback = MoveToPose.Feedback()
        result = MoveToPose.Result()
        self.set_planning_settings(goal_handle)
        try:
            result.success = self.move_to_pose(goal_handle, feedback)
            self.get_logger().info(
                "Move to pose finished with result: " + str(result.success)
            )
            goal_handle.succeed()
            self.reset_planning_settings(goal_handle)
            return result
        except Exception as e:
            self.get_logger().error(f"Move to pose failed: {str(e)}")
            goal_handle.succeed()
            self.reset_planning_settings(goal_handle)
            result.success = False
            return result

    def move_joints_execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing joint goal...")

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
            goal_handle.succeed()
            result.success = False
            return result

    def move_to_pose(self, goal_handle, feedback):
        """Perform the pick operation."""
        pose = goal_handle.request.pose
        target_link = goal_handle.request.target_link
        tolerance_position = goal_handle.request.tolerance_position if goal_handle.request.tolerance_position else 0.01
        tolerance_orientation = goal_handle.request.tolerance_orientation if goal_handle.request.tolerance_orientation else 0.05
        if target_link != "":
            result = self.planner.plan_pose_goal(
                pose=pose,
                target_link=target_link,
                wait=True,
                set_mode=True,
                tolerance_position=tolerance_position,
                tolerance_orientation=tolerance_orientation,
            )
        else:
            result = self.planner.plan_pose_goal(
                pose=pose,
                wait=True,
                set_mode=(self.current_mode != MOVEIT_MODE),
                tolerance_position=tolerance_position,
                tolerance_orientation=tolerance_orientation,
            )
        if not ALWAYS_SET_MODE:
            self.current_mode = MOVEIT_MODE
        return result

    def move_joints(self, goal_handle, feedback):
        self.get_logger().info(
            f"Moving joints: {goal_handle.request.joint_names} to : {list(goal_handle.request.joint_positions)}"
        )
        joint_names = goal_handle.request.joint_names
        joint_positions = list(goal_handle.request.joint_positions)
        joint_dict = self.planner.get_joint_positions()
        if len(list(joint_dict.keys())) == 0:
            self.get_logger().error("No joints available")
            return False
        try:
            configuration_distance = 0
            for i, joint_name in enumerate(joint_names):
                # self.get_logger().info(
                #     f"Joint name: {joint_name}, position: {joint_positions[i]}, index: {i}"
                # )
                joint_curr_pos = joint_dict[joint_name]
                joint_target_pos = joint_positions[i]
                configuration_distance += (joint_curr_pos - joint_target_pos) ** 2
            configuration_distance = configuration_distance**0.5
            # self.get_logger().info(
            #     f"Joint configuration distance: {configuration_distance}"
            # )
            if configuration_distance < MIN_CONFIGURATION_DISTANCE_TRESHOLD:
                self.get_logger().info(
                    f"Joint positions are already close to target: {configuration_distance}"
                )
                return True
        except Exception as e:
            self.get_logger().error(str(e))
            return False

        self.get_logger().info("Planning joint goal...")
        result = self.planner.plan_joint_goal(
            joint_positions,
            joint_names,
            wait=True,
            set_mode=(self.current_mode != MOVEIT_MODE),
        )
        self.get_logger().info(f"Move Joints Result: {result}")
        if not ALWAYS_SET_MODE:
            self.current_mode = MOVEIT_MODE
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
            else PICK_PLANNER
        )
        try:
            planning_time = (
                goal_handle.request.planning_time
                if goal_handle.request.planning_time > 0.1
                else 0.5
            )
            planning_attempts = (
                goal_handle.request.planning_attempts
                if goal_handle.request.planning_attempts > 0
                else 5
            )
        except Exception as e:
            self.get_logger().error(f"Error setting planning time: {str(e)}")
            planning_time = 0.5
            planning_attempts = 5
        
        self.planner.set_velocity(velocity)
        self.planner.set_acceleration(acceleration)
        self.planner.set_planner(planner_id)
        self.get_logger().info(
            f"Planning settings: velocity={velocity}, acceleration={acceleration}, planner_id={planner_id}, planning_time={planning_time}"
        )
        self.planner.set_planning_time(planning_time)
        self.planner.set_planning_attempts(planning_attempts)

        if goal_handle.request.apply_constraint:
            self.get_logger().info("Planning with Constraints...")
            self.planner.set_orientation_constraints(goal_handle.request)

    def reset_planning_settings(self, goal_handle):
        if goal_handle.request.apply_constraint:
            self.get_logger().info("Deleting all constraints...")
            self.planner.delete_all_constraints()

    def get_joints_callback(self, request, response):
        joint_dict = self.planner.get_joint_positions()
        for joint_name in joint_dict:
            response.joint_positions.append(float(joint_dict[joint_name]))
            response.joint_names.append(joint_name)
        return response

    def add_collision_objects_callback(self, request, response):
        """Handle requests to add collision objects to the planning scene"""
        for collision_object in request.collision_objects:
            try:
                # Generate a unique ID for the collision object
                object_id = f"{collision_object.id}"

                # Handle different collision object types
                if collision_object.type == "box":
                    self.planner.add_collision_box(
                        id=object_id,
                        size=(
                            collision_object.dimensions.x,
                            collision_object.dimensions.y,
                            collision_object.dimensions.z,
                        ),
                        pose=collision_object.pose,
                    )
                    # self.get_logger().info(f"Added collision box: {object_id}")

                elif collision_object.type == "sphere":
                    # For spheres, use the x component of dimensions as radius
                    self.planner.add_collision_sphere(
                        id=object_id,
                        radius=collision_object.dimensions.x,
                        pose=collision_object.pose,
                    )
                    # self.get_logger().info(f"Added collision sphere: {object_id}")

                elif collision_object.type == "cylinder":
                    # For cylinders, use x as radius, z as height
                    self.planner.add_collision_cylinder(
                        id=object_id,
                        height=collision_object.dimensions.z,
                        radius=collision_object.dimensions.x,
                        pose=collision_object.pose,
                    )
                    # self.get_logger().info(f"Added collision cylinder: {object_id}")

                elif collision_object.type == "mesh":
                    # Add collision mesh from file path -> Priority to file path
                    if collision_object.path_to_mesh:
                        self.planner.add_collision_mesh(
                            id=object_id,
                            filepath=collision_object.path_to_mesh,
                            pose=collision_object.pose,
                            scale=(
                                collision_object.dimensions.x
                                if collision_object.dimensions.x != 0.0
                                else 1.0
                            ),
                        )
                        # self.get_logger().info(
                        #     f"Added collision mesh from file: {object_id}"
                        # )
                    # Or from mesh data
                    else:
                        import trimesh

                        # Convert mesh data to trimesh object
                        mesh = trimesh.Trimesh()
                        mesh.vertices = [
                            (v.x, v.y, v.z) for v in collision_object.mesh.vertices
                        ]
                        mesh.faces = [
                            (
                                t.vertex_indices[0],
                                t.vertex_indices[1],
                                t.vertex_indices[2],
                            )
                            for t in collision_object.mesh.triangles
                        ]

                        # transform from
                        self.planner.add_collision_mesh(
                            id=object_id,
                            filepath=None,
                            pose=collision_object.pose,
                            mesh=mesh,
                            frame_id=collision_object.pose.header.frame_id,
                        )
                        # self.get_logger().info(
                        #     f"Added collision mesh from data: {object_id}"
                        # )
                else:
                    self.get_logger().error(
                        f"Unsupported collision object type: {collision_object.type}"
                    )
                    response.success = False
                    return response

                response.success = True

            except Exception as e:
                self.get_logger().error(f"Failed to add collision object: {str(e)}")
                response.success = False
        self.get_logger().info("Finished adding objects")
        return response

    def attach_collision_object_callback(self, request, response):
        """Handle requests to attach collision objects to the planning scene"""
        attached_id = f"{request.id}"
        if request.detach:
            self.planner.detach_collision_object(attached_id)
            self.get_logger().info(f"Detached collision object: {attached_id}")
            response.success = True
            return response
        attached_link = request.attached_link
        touch_links = request.touch_links
        self.planner.attach_collision_object(attached_id, attached_link, touch_links)
        self.get_logger().info(f"Attached collision object: {attached_id}")
        response.success = True
        return response

    def remove_collision_object_callback(self, request, response):
        """Handle requests to remove collision objects from the planning scene"""
        try:
            # Generate a unique ID for the collision object
            object_id = f"{request.id}"
            if object_id == "all":
                self.planner.remove_all_collision_objects(
                    include_attached=request.include_attached
                )
                self.get_logger().info("Removed all collision objects")
                response.success = True
                return response

            # Remove the collision object
            self.planner.remove_collision_object(object_id)
            self.get_logger().info(f"Removed collision object: {object_id}")

            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to remove collision object: {str(e)}")
            response.success = False
            return response

    def toggle_servo_callback(self, request, response):
        """Handle requests to toggle the servo"""
        try:
            if request.state:
                self.servo.enable_servo()
                self.get_logger().info("Enabled servo")
            else:
                self.servo.disable_servo()
                self.get_logger().info("Disabled servo")
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to toggle servo: {str(e)}")
            response.success = False
            return response

    def servo_speed_callback(self, msg):
        """Handle changes to the servo speed"""
        self.get_logger().info(f"Received servo speed: {msg.twist}")

        self.servo.update_servo(
            linear=(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
            angular=(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z),
            frame_id=msg.header.frame_id,
        )

    def set_joint_velocity_callback(self, request, response):
        velocities = request.speeds
        success = self.xarm_services.set_joint_velocity(
            velocities, set_mode=(self.current_mode != JOINT_VELOCITY_MODE)
        )
        # if we always want to change mode, never set it to joint velocity mode
        if not ALWAYS_SET_MODE:
            self.current_mode = JOINT_VELOCITY_MODE
        response.success = success

    def get_collision_objects_callback(self, request, response):
        """Handle requests to get the collision objects in the planning scene"""
        response = GetCollisionObjects.Response()

        self.planner.update_planning_scene()
        planning_scene = self.planner.get_planning_scene()
        print(planning_scene)
        for collision_object in planning_scene.world.collision_objects:
            new_collision_object = CollisionObject()
            new_collision_object.id = collision_object.id
            new_collision_object.pose.header = collision_object.header
            new_collision_object.pose.pose = collision_object.pose
            # TODO: send dimensions and type
            response.collision_objects.append(new_collision_object)
        response.success = True
        return response

    def set_gripper_state_callback(self, request, response):
        """Handle requests to set the gripper state"""
        self.get_logger().info(f"Setting gripper state: {request.data}")
        # 0 open, 1 closed
        if request.data:
            self.xarm_services.open_gripper()
        else:
            self.xarm_services.close_gripper()
        response.success = True

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
