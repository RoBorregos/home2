""" Map tagger application script.

Application for manual map contextualization done by tagging the map
with the "2D Goal Pose" rviz option. 

The user will use rviz and the terminal to define the area of rooms 
and objects of interest in the map. For this, please run the map_tagger
launch file and follow the app menu.

Global app states:
1. Add room entrance(s)
2. Add room area
3. Add room path
4. Add object of interest
5. Save current map to YAML
6. Load map from YAML
7. Add global hallway/external path
8. State currrent room number
9. State currrent global path number
0. Do nothing / wait
-1. Exit


TO USE:
 build map_context_tests and map_context_interfaces packages:
    colcon build --packages-select map_context_tests map_context_interfaces
 
 Launch "map_tagger.launch.py" launch file (it will load the rviz config file and 
    open terminal with the menu) 
    
    ros2 launch map_context_tests map_tagger.launch.py

 or run "map_tagger.py" and launch rviz separately


EXAMPLE:
 select option 6 and write "home_example5" to view an example

Notes: 
- The order of the 2D goal poses must be selected counter clockwise, or the side with color will be looking "down" (idk why)  
- if it breaks, restart / relaunch 

Pending tasks:
- FRIDA Specific Pose saver
- Not all possibilities go back to the "promt_state" menu, it still breaks sometimes
- Maybe calculate dimensions of rooms/objects
- fix note
- ... 
"""

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Point
from map_context_interfaces.msg import MapContext, Room, ObjInt, GlobalPath

from visualization_msgs.msg import Marker, MarkerArray

import threading
import colorsys
import yaml
import os
from pathlib import Path


class MapTagger(Node):
    def __init__(self):
        super().__init__('map_tagger')

        self.STATE_WAIT = 0
        self.STATE_ENTRANCE = 1
        self.STATE_AREA = 2
        self.STATE_PATH = 3
        self.STATE_OBJECT = 4
        self.STATE_SAVE_MAP = 5
        self.STATE_LOAD_MAP = 6
        self.STATE_GLOBAL_PATH = 7  # external paths like hallways
        self.STATE_SELECT_ROOM = 8
        self.STATE_SELECT_GLOBAL_PATH = 9
        self.STATE_DONE = -1

        self.CURRENT_ROOM = -1
        self.CURRENT_GLOBAL_PATH = -1          
        self.global_path_points_read = False
        self.global_path_point_count = 0
        self.current_global_path_name = ""

        self.state = self.STATE_WAIT

        self.path_point_num = 0
        self.saved_file_name = ""
        self.clicked_points = []
        self.map = MapContext()
        self.map.name = "semantic_map"
        self.label_pub = self.create_publisher(MarkerArray, "room_labels", 10)
        self.map_pub = self.create_publisher(MapContext, "tagged_map", 10)
        
        self.data_path = None
        self.find_data_path()  

        # RVIZ visualization MarkerArray topic publishers
        self.marker_pub = self.create_publisher(MarkerArray, "room_marker", 10)
        self.entrance_pub = self.create_publisher(MarkerArray, 'room_entrances_markers', 10)
        self.path_pub = self.create_publisher(MarkerArray, 'room_paths_markers', 10)
        self.global_path_pub = self.create_publisher(MarkerArray, 'global_paths_markers', 10)
        self.obj_pub = self.create_publisher(MarkerArray, 'room_objects_markers', 10)

        self.sub = self.create_subscription(PoseStamped, '/goal_pose', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        threading.Thread(target=self.prompt_state, daemon=True).start()

    def timer_callback(self):
        self.publish_all_markers()
        self.publish_features()
        self.map_pub.publish(self.map)

    def prompt_state(self):
        if not rclpy.ok():  
          return
    
        print("\n🌐 Map Tagger - Choose what to tag:")
        print("1. Add room entrance(s)")
        print("2. Add room area")
        print("3. Add room path")
        print("4. Add object of interest")
        print("5. Save current map to YAML")
        print("6. Load map from YAML")
        print("7. Add global hallway/external path")
        print("8. State currrent room number to edit")
        print("9. Edit specific global path")
        print("0. Do nothing / wait")
        print("-1. Exit")

        try:
            selection = int(input("Enter a state number: "))
        except ValueError:
            selection = 0

        self.state = selection
        print(f"🧭  Current state: {self.state}")       


        if (self.state == self.STATE_ENTRANCE):
            print("Select Entrance Point: ")
        elif (self.state == self.STATE_AREA):
            print("Select 4 points for room area")
        elif (self.state == self.STATE_PATH):
            try:
                self.remaining_path_points = int(input("Desired number of path points: ").strip())
            except ValueError:
                print("❌ Invalid number, defaulting to 1")
                self.remaining_path_points = 1
            print(f"📌 Select {self.remaining_path_points} Path Point(s):")
        elif (self.state == self.STATE_OBJECT):
            print("📌 Select 4 points for ObjInt area")
        elif (self.state == self.STATE_SAVE_MAP):
            self.save_current_map()
        elif (self.state == self.STATE_LOAD_MAP):
            self.load_map_from_yaml()
        elif (self.state == self.STATE_GLOBAL_PATH):
            global_path = GlobalPath()
            try:
                global_path.name = input("Name global path: ").strip()
                self.global_path_point_count = int(input("How many global path points to select: ").strip())
                global_path.num_points = self.global_path_point_count
            except ValueError:
                self.global_path_point_count = 1
            print(f"🌐 Select {self.global_path_point_count} point(s) for the {self.current_global_path_name} global path")
            self.map.global_paths.append(global_path)
            self.CURRENT_GLOBAL_PATH = -1
        elif self.state == self.STATE_SELECT_ROOM:
            self.select_room_to_edit()
        elif self.state == self.STATE_SELECT_GLOBAL_PATH:
            self.select_global_path_to_edit()
            
        elif (self.state == self.STATE_DONE):
            self.get_logger().info("✅  Tagging complete.")
            ######################## EXIT PROGRAM ########################
            self.destroy_node()
            rclpy.get_default_context().shutdown()  # Use this safe form
            return
        else: 
            print(f"🔢  Active thread count: {threading.active_count()}")
            # for t in threading.enumerate():
            #     print(f"🧵 Thread: {t.name}, Alive: {t.is_alive()}, Daemon: {t.daemon}")
            self.state = self.STATE_WAIT
            self.get_logger().info("⏸️  Waiting... (state 0)")
            threading.Thread(target=self.prompt_state, daemon=True).start()   

    def select_global_path_to_edit(self):
        if not self.map.global_paths:
            print("⚠️ No global paths available.")
            threading.Thread(target=self.prompt_state, daemon=True).start()
            return

        print("\n🌐 Global Paths:")
        for i, gp in enumerate(self.map.global_paths):
            print(f"{i}: {gp.name} ({len(gp.path)} point(s))")

        try:
            index = int(input("Enter the index of the global path to edit: "))
            if 0 <= index < len(self.map.global_paths):
                self.CURRENT_GLOBAL_PATH = index
                self.global_path_point_count = int(input("How many points to add to this global path? "))
                print(f"✅ Editing global path: '{self.map.global_paths[index].name}', select {self.global_path_point_count} new point(s)")
                self.state = self.STATE_GLOBAL_PATH  # reuse global path logic
            else:
                print("❌ Invalid index.")
                threading.Thread(target=self.prompt_state, daemon=True).start()
        except ValueError:
            print("❌ Invalid input.")
            threading.Thread(target=self.prompt_state, daemon=True).start()


    def select_room_to_edit(self):
        if not self.map.rooms:
            print("⚠️ No rooms available.")
            threading.Thread(target=self.prompt_state, daemon=True).start()
            return

        print("\n🏘️ Rooms:")
        for i, room in enumerate(self.map.rooms):
            print(f"{i}: {room.name}")
        try:
            index = int(input("Enter the index of the room to edit: "))
            if 0 <= index < len(self.map.rooms):
                self.current_room_index = index
                print(f"✅ Current room: '{self.map.rooms[index].name}'")
            else:
                print("❌ Invalid index.")
        except ValueError:
            print("❌ Invalid input.")
        
        threading.Thread(target=self.prompt_state, daemon=True).start()


    def save_current_map(self):
        print("\n To save current Map, enter a desired YAML file name:")
        self.saved_file_name = input("File name: ").strip()
        self.save_map_to_yaml(self.map, self.saved_file_name)
        self.get_logger().info(f"📝  Map saved to {self.saved_file_name}")
        threading.Thread(target=self.prompt_state, daemon=True).start()   

    def pose_callback(self, msg):
        point = msg.pose.position
        self.get_logger().info(f"📍 Clicked: ({point.x:.2f}, {point.y:.2f})")
        # self.get_logger().info(f"🏠 Total rooms: {len(self.map.rooms)}")

        if self.state == self.STATE_ENTRANCE:
            if not self.map.rooms:
                print("⚠️  No room exists yet. Please define a room first.")
                threading.Thread(target=self.prompt_state, daemon=True).start()
                return
            self.map.rooms[self.CURRENT_ROOM].entrances.append(point)
            self.get_logger().info("➕ Added entrance point.")

        elif self.state == self.STATE_AREA:
            self.clicked_points.append(point)
            self.get_logger().info(f"🧱  Point {len(self.clicked_points)} for area.")
            if len(self.clicked_points) == 4:
                threading.Thread(target=self.prompt_room_name, daemon=True).start()
            return

        elif self.state == self.STATE_PATH:
            if not self.map.rooms:
                print("⚠️  No room exists yet.")
                threading.Thread(target=self.prompt_state, daemon=True).start()
                return
            self.map.rooms[self.CURRENT_ROOM].path.append(point)
            self.remaining_path_points -= 1
            self.get_logger().info(f"➕ Added path point ({len(self.map.rooms[self.CURRENT_ROOM].path)} total), remaining: {self.remaining_path_points}")

            if self.remaining_path_points <= 0:
                threading.Thread(target=self.prompt_state, daemon=True).start()
            return
            

        elif self.state == self.STATE_OBJECT:
            self.clicked_points.append(point)
            self.get_logger().info(f"🧱  Point {len(self.clicked_points)} for object of interest.")
            if len(self.clicked_points) == 4:
                threading.Thread(target=self.prompt_object_name, daemon=True).start()
            return

        elif self.state == self.STATE_GLOBAL_PATH:
            self.map.global_paths[self.CURRENT_GLOBAL_PATH].path.append(point)
            self.global_path_point_count -= 1
            self.get_logger().info(f"🌐 Added global path point, remaining: {self.global_path_point_count}")

            if self.global_path_point_count <= 0:
                threading.Thread(target=self.prompt_state, daemon=True).start()
                self.global_path_points_read = True
            return

        elif self.state == self.STATE_SAVE_MAP:
            return
        
        elif self.state == self.STATE_LOAD_MAP:
            return

        elif self.state == self.STATE_DONE:
            return
        
        # else:
        #     return
        
        threading.Thread(target=self.prompt_state, daemon=True).start()
    

    def prompt_object_name(self):
        print("\n🔷 You clicked 4 points for an object. Enter object name:")
        obj_name = input("Object name: ").strip()
        if not obj_name:
            print("❌ Object name cannot be empty.")
            self.clicked_points.clear()
            return

        obj = ObjInt()
        obj.name = obj_name
        obj.obj_area = self.clicked_points.copy()
        self.map.rooms[self.CURRENT_ROOM].obj_int.append(obj)
        self.clicked_points.clear()

        self.get_logger().info(f"✅ Object '{obj_name}' saved to room '{self.map.rooms[-1].name}'")
        threading.Thread(target=self.prompt_state, daemon=True).start()

    def prompt_room_name(self):
        print("\n🧠 You clicked 4 points. Enter a room name:")
        room_name = input("Room name: ").strip()

        confirm = input("Press [s] to save, [r] to reset: ").strip().lower()
        if confirm == 's':
            self.save_room(room_name)
        else:
            print("❌ Resetting clicked points.")
            self.clicked_points.clear()
        
        threading.Thread(target=self.prompt_state, daemon=True).start()

    def save_room(self, name):
        room = Room()
        room.name = name
        room.area = self.clicked_points.copy()
        self.map.rooms.append(room)
        self.clicked_points.clear()

        # self.map_pub.publish(self.map)
        self.get_logger().info(f"✅ Room '{name}' saved and published.")

    def publish_all_markers(self):
        marker_array = MarkerArray()
        label_array = MarkerArray()
        z_offset = 0.05

        for i, room in enumerate(self.map.rooms):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "room_filled"
            marker.id = i
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.lifetime.sec = 0

            r, g, b = self.get_distinct_color(i, len(self.map.rooms))
            marker.color.r, marker.color.g, marker.color.b = r, g, b
            marker.color.a = 0.9

            def elevated(p): return Point(x=p.x, y=p.y, z=z_offset)
            pts = room.area.copy()
            if len(pts) >= 3:
                for j in range(1, len(pts) - 1):
                    marker.points.append(elevated(pts[0]))
                    marker.points.append(elevated(pts[j]))
                    marker.points.append(elevated(pts[j + 1]))

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "room_labels"
            label.id = 1000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.pose.position.x = sum(pt.x for pt in pts) / len(pts)
            label.pose.position.y = sum(pt.y for pt in pts) / len(pts)
            label.pose.position.z = 1.5
            label.pose.orientation.w = 1.0
            label.text = room.name

            marker_array.markers.append(marker)
            label_array.markers.append(label)

        # === Global Path Labels ===
        for i, gp in enumerate(self.map.global_paths):
            if len(gp.path) == 0:
                continue  # Can't label empty path

            # Compute average position for the label
            avg_x = sum(p.x for p in gp.path) / len(gp.path)
            avg_y = sum(p.y for p in gp.path) / len(gp.path)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "room_labels"  # same namespace as rooms
            label.id = 2000 + i  # avoid collision with room labels (1000 + i)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.pose.position.x = avg_x
            label.pose.position.y = avg_y
            label.pose.position.z = 1.5
            label.pose.orientation.w = 1.0
            label.text = gp.name

            label_array.markers.append(label)

        self.marker_pub.publish(marker_array)
        self.label_pub.publish(label_array)

    def publish_features(self):
        entrances = MarkerArray()
        paths = MarkerArray()
        global_paths = MarkerArray()
        objects = MarkerArray()

        entrance_id = 0
        path_id = 0
        obj_id = 0

        for i, room in enumerate(self.map.rooms):
            # === Entrances ===
            for j, pt in enumerate(room.entrances):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "room_entrances"
                marker.id = entrance_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.scale.x = marker.scale.y = 0.15
                marker.scale.z = 0.3
                marker.color.r = 0.6
                marker.color.g = 0.1
                marker.color.b = 0.9
                marker.color.a = 1.0
                marker.pose.position = Point(x=pt.x, y=pt.y, z=0.15)
                marker.pose.orientation.w = 1.0
                entrances.markers.append(marker)
                entrance_id += 1

            # === Paths ===
            for j, pt in enumerate(room.path):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "room_paths"
                marker.id = path_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = marker.scale.y = marker.scale.z = 0.12
                marker.color.r = 0.1
                marker.color.g = 0.7
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.pose.position = Point(x=pt.x, y=pt.y, z=0.1)
                marker.pose.orientation.w = 1.0
                paths.markers.append(marker)
                path_id += 1

            # === Objects of Interest ===
            for obj in room.obj_int:
                if len(obj.obj_area) < 3:
                    continue  # Skip if not enough points

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "room_objects"
                marker.id = obj_id
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.ADD
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.4
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.orientation.w = 1.0
                marker.lifetime.sec = 0

                # Prism generation from base polygon
                def elevated(p, z): return Point(x=p.x, y=p.y, z=z)
                base = obj.obj_area
                top_z = 0.3
                bot_z = 0.0
                n = len(base)

                # Top face
                for i in range(1, n - 1):
                    marker.points.append(elevated(base[0], top_z))
                    marker.points.append(elevated(base[i], top_z))
                    marker.points.append(elevated(base[i + 1], top_z))

                # Bottom face
                for i in range(1, n - 1):
                    marker.points.append(elevated(base[0], bot_z))
                    marker.points.append(elevated(base[i + 1], bot_z))
                    marker.points.append(elevated(base[i], bot_z))

                # Side walls
                for i in range(n):
                    a = base[i]
                    b = base[(i + 1) % n]
                    a_bot = elevated(a, bot_z)
                    b_bot = elevated(b, bot_z)
                    a_top = elevated(a, top_z)
                    b_top = elevated(b, top_z)

                    marker.points.append(a_top)
                    marker.points.append(a_bot)
                    marker.points.append(b_top)

                    marker.points.append(b_top)
                    marker.points.append(a_bot)
                    marker.points.append(b_bot)

                objects.markers.append(marker)
                obj_id += 1

        # === Global Path ===
        global_paths.markers.clear()

        for i, gp in enumerate(self.map.global_paths):
            if len(gp.path) < 2:
                continue  # Need at least 2 points to draw a surface

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"global_path_{gp.name}"
            marker.id = i
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0

            width = 0.7  # Width of the "road" in meters
            z_elevation = 0.0  # Small lift from ground

            for j in range(len(gp.path) - 1):
                p1 = gp.path[j]
                p2 = gp.path[j+1]

                # Calculate the perpendicular vector (normal) to the segment (p2 - p1)
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                length = (dx**2 + dy**2)**0.5
                if length == 0:
                    continue  # skip degenerate segments

                # Unit normal vector to the left (90 degrees rotation)
                nx = -dy / length
                ny = dx / length

                # Offset points to the left and right
                p1_left  = Point(x=p1.x + nx * width/2, y=p1.y + ny * width/2, z=z_elevation)
                p1_right = Point(x=p1.x - nx * width/2, y=p1.y - ny * width/2, z=z_elevation)
                p2_left  = Point(x=p2.x + nx * width/2, y=p2.y + ny * width/2, z=z_elevation)
                p2_right = Point(x=p2.x - nx * width/2, y=p2.y - ny * width/2, z=z_elevation)

                # Form two triangles (quad made of two triangles)
                marker.points.append(p1_left)
                marker.points.append(p1_right)
                marker.points.append(p2_left)

                marker.points.append(p2_left)
                marker.points.append(p1_right)
                marker.points.append(p2_right)

            global_paths.markers.append(marker)
            
        self.entrance_pub.publish(entrances)
        self.path_pub.publish(paths)
        self.global_path_pub.publish(global_paths)
        self.obj_pub.publish(objects)

    def find_nearest_point(self, reference_point, candidates):
        """Return the point in `candidates` closest to `reference_point`."""
        if not candidates:
            return reference_point
        return min(candidates, key=lambda p: (p.x - reference_point.x)**2 + (p.y - reference_point.y)**2)

    def get_distinct_color(self, i, total):
        hue = i / total
        r, g, b = colorsys.hsv_to_rgb(hue, 0.75, 1.0)
        return r, g, b

    def save_map_to_yaml(self, map_msg, filename="map_output.yaml", directory=""):
        # Ensure the directory exists
        
        directory = self.data_path

        if not filename.endswith(".yaml"):
            filename += ".yaml"

        # Build full path
        full_path = os.path.join(directory, filename)

        map_dict = self.mapcontext_to_dict(map_msg)
        with open(full_path, "w") as f:
            yaml.dump(map_dict, f, sort_keys=False)

    def mapcontext_to_dict(self, map_msg):
        def point_to_dict(p):
            return {'x': p.x, 'y': p.y, 'z': p.z}

        def objint_to_dict(obj):
            return {
                'name': obj.name,
                'obj_area': [point_to_dict(p) for p in obj.obj_area]
            }

        def room_to_dict(room):
            return {
                'name': room.name,
                'area': [point_to_dict(p) for p in room.area],
                'path': [point_to_dict(p) for p in room.path],
                'entrances': [point_to_dict(p) for p in room.entrances],
                'obj_int': [objint_to_dict(obj) for obj in room.obj_int]
            }

        return {
            'name': map_msg.name,
            'rooms': [room_to_dict(r) for r in map_msg.rooms],
            'global_paths': [
                {
                    'name': gp.name,
                    'num_points': gp.num_points,
                    'path': [point_to_dict(p) for p in gp.path]
                } for gp in map_msg.global_paths
            ]
        }

    def load_map_from_yaml(self):
        print("\n📂 Enter the YAML file name to load (from 'data' folder):")
        filename = input("File name: ").strip()
        if not filename.endswith(".yaml"):
            filename += ".yaml"

        # package_path = get_package_directory('map_context_tests')
        full_path = os.path.join(self.data_path, filename)

        if not os.path.exists(full_path):
            print(f"❌ File '{full_path}' not found.")
            threading.Thread(target=self.prompt_state, daemon=True).start()
            return

        try:
            with open(full_path, "r") as f:
                map_dict = yaml.safe_load(f)

            # Build MapContext from dictionary
            self.map = MapContext()
            self.map.name = map_dict.get("name", "loaded_map")

            def dict_to_point(d): return Point(x=d["x"], y=d["y"], z=d["z"])

            for room_data in map_dict.get("rooms", []):
                room = Room()
                room.name = room_data.get("name", "default_room_name")
                room.area = [dict_to_point(p) for p in room_data.get("area", [])]
                room.path = [dict_to_point(p) for p in room_data.get("path", [])]
                room.entrances = [dict_to_point(p) for p in room_data.get("entrances", [])]

                for obj_data in room_data.get("obj_int", []):
                    obj = ObjInt()
                    obj.name = obj_data.get("name", "default_objint_name")
                    obj.obj_area = [dict_to_point(p) for p in obj_data.get("obj_area", [])]
                    room.obj_int.append(obj)

                self.map.rooms.append(room)

            # ✅ Load global path points (outside rooms)
            for gp_data in map_dict.get("global_paths", []):
                gp = GlobalPath()
                gp.name = gp_data.get("name", "unnamed_path")
                gp.num_points = gp_data.get("num_points", 0)
                gp.path = [dict_to_point(p) for p in gp_data.get("path", [])]
                self.map.global_paths.append(gp)

            self.get_logger().info(f"✅ Map loaded from {filename} with {len(self.map.rooms)} room(s) and {len(self.map.global_paths)} global path point(s).")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to load map: {e}")

        threading.Thread(target=self.prompt_state, daemon=True).start()

    def find_data_path(self):
        # Candidate paths in priority order
        candidate_paths = [
            Path("src/navigation/packages/map_context_tests/data"),
            Path("navigation/packages/map_context_tests/data")
        ]

        for path in candidate_paths:
            if path.exists():
                self.data_path = str(path)
                self.get_logger().info(f"✅ Found data path at: {self.data_path}")
                return

        # If none found, ask the user
        while True:
            user_path = input("❓ Could not find data folder. Please input the path manually: ").strip()
            path = Path(user_path)
            if path.exists() and path.is_dir():
                self.data_path = str(path)
                self.get_logger().info(f"✅ Using manually provided data path: {self.data_path}")
                return
            else:
                print("❌ Invalid path. Please try again.")

def main(args=None):
    rclpy.init(args=args)
    node = MapTagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
