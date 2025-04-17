#!/usr/bin/env python3

"""
Task Manager for Storing Groceries task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3


class StoringGroceriesTM(Node):
    """Class to manage the Storing Groceries task"""

    TASK_STATES = {
        "START": 0,
        "NAVIGATE_TO_TABLE": 1,
        "PERCEIVE_TABLE_OBJECTS": 2,
        "PICK_OBJECT": 3,
        "NAVIGATE_TO_CABINET": 4,
        "PERCEIVE_CABINET_OBJECTS": 5,
        "DETERMINE_PLACEMENT_LAYER": 6,
        "PLACE_OBJECT": 7,
        "OPEN_FIRST_CABINET": 8,
        "OPEN_SECOND_CABINET": 9,
        "POUR_CEREAL": 10,
        "END": 11,
    }

    # Deus Ex Machina penalty values
    DEM_PENALTIES = {
        "HUMAN_HANDOVER": -50,
        "HUMAN_PLACE": -15,
        "HUMAN_PLACE_SIMILAR": -50,
        "HUMAN_POINTING": -25,
        "HUMAN_OPEN_CABINET": -100,
        "SPILL_CEREAL": -100,
        "LEAVE_CEREAL": -100,
        "HUMAN_POUR_CEREAL": -300
    }

    def _init_(self):
        """Initialize the node"""
        super()._init_("storing_groceries_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.STORING_GROCERIES, mock_areas=["manipulation", "navigation"]
        )
        self.current_state = StoringGroceriesTM.TASK_STATES["START"]
        self.current_attempts = 0
        self.running_task = True
        self.objects_to_organize = []
        self.current_object = None
        self.current_object_category = None
        self.cabinet_layers = {}  # Map of categories to layers
        self.cereal_found = False
        self.cereal_container_found = False
        
        # Penalty tracking
        self.total_score = 0
        self.penalties = {
            "HUMAN_HANDOVER": 0,
            "HUMAN_PLACE": 0,
            "HUMAN_PLACE_SIMILAR": 0,
            "HUMAN_POINTING": 0,
            "HUMAN_OPEN_CABINET": 0,
            "SPILL_CEREAL": 0,
            "LEAVE_CEREAL": 0,
            "HUMAN_POUR_CEREAL": 0
        }

        Logger.info(self, "StoringGroceriesTaskManager has started.")

    def navigate_to(self, location: str, sublocation: str = ""):
        """Navigate to a specific location"""
        self.subtask_manager.hri.say(f"I will navigate to {location}")
        self.subtask_manager.manipulation.move_to_position("navigation")
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def categorize_object(self, object_detection):
        """Determine category of detected object"""
        # This would use vision APIs to determine category based on detected features
        categories = {
            "cereals": ["cereal box", "oatmeal", "granola"],
            "canned_goods": ["soup can", "beans can", "tuna can"],
            "spices": ["salt", "pepper", "oregano", "cinnamon"],
            "containers": ["bowl", "container", "jar", "tupperware"],
            "utensils": ["spoon", "fork", "knife", "spatula"]
        }
        
        # Mock implementation - would be replaced with actual vision classification
        for category, items in categories.items():
            if object_detection['label'].lower() in items:
                return category
        
        return "miscellaneous"

    def determine_cabinet_layer(self, category):
        """Determine which layer in the cabinet this category belongs to"""
        # Predefined mapping of categories to cabinet layers
        layer_mapping = {
            "cereals": "top",
            "canned_goods": "middle",
            "spices": "middle",
            "containers": "bottom",
            "utensils": "drawer",
            "miscellaneous": "bottom"
        }
        
        return layer_mapping.get(category, "middle")
    
    def apply_penalty(self, penalty_type):
        """Apply a specific penalty and track it"""
        if penalty_type in self.penalties and self.penalties[penalty_type] < 5:
            # Most penalties are limited to 5 instances
            self.penalties[penalty_type] += 1
            penalty_value = self.DEM_PENALTIES[penalty_type]
            self.total_score += penalty_value
            Logger.info(self, f"Applied {penalty_type} penalty: {penalty_value}. Total score: {self.total_score}")
            return True
        elif penalty_type == "HUMAN_OPEN_CABINET" and self.penalties[penalty_type] < 2:
            # Cabinet opening is limited to 2 instances
            self.penalties[penalty_type] += 1
            penalty_value = self.DEM_PENALTIES[penalty_type]
            self.total_score += penalty_value
            Logger.info(self, f"Applied {penalty_type} penalty: {penalty_value}. Total score: {self.total_score}")
            return True
        elif penalty_type in ["SPILL_CEREAL", "LEAVE_CEREAL", "HUMAN_POUR_CEREAL"]:
            # These penalties can only be applied once
            if self.penalties[penalty_type] == 0:
                self.penalties[penalty_type] = 1
                penalty_value = self.DEM_PENALTIES[penalty_type]
                self.total_score += penalty_value
                Logger.info(self, f"Applied {penalty_type} penalty: {penalty_value}. Total score: {self.total_score}")
                return True
        
        # Return False if penalty limit reached
        return False

    def request_human_handover(self):
        """Request human to hand over an object to the robot"""
        if self.apply_penalty("HUMAN_HANDOVER"):
            self.subtask_manager.hri.say("I'm having trouble picking up the object. Could you please hand it to me?")
            self.subtask_manager.manipulation.prepare_for_handover()
            
            # Wait for confirmation of handover
            while not self.subtask_manager.hri.confirm("Have you handed me the object?", timeout=30):
                self.subtask_manager.hri.say("Please hand me the object when ready.")
            
            self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say("Thank you for your assistance.")
            return Status.EXECUTION_SUCCESS
        else:
            self.subtask_manager.hri.say("I've already received maximum assistance with handovers.")
            return Status.EXECUTION_FAILURE

    def request_human_placement(self, is_similar_objects=False):
        """Request human to place the object in the cabinet"""
        penalty_type = "HUMAN_PLACE_SIMILAR" if is_similar_objects else "HUMAN_PLACE"
        
        if self.apply_penalty(penalty_type):
            message = (
                "I'm having trouble placing this object next to similar items. Could you help place it?"
                if is_similar_objects
                else "I'm having trouble placing this object. Could you help place it in the cabinet?"
            )
            
            self.subtask_manager.hri.say(message)
            self.subtask_manager.manipulation.open_gripper()
            
            # Wait for confirmation of placement
            while not self.subtask_manager.hri.confirm("Have you placed the object?", timeout=30):
                self.subtask_manager.hri.say("Please take the object and place it when ready.")
            
            self.subtask_manager.hri.say("Thank you for your assistance.")
            return Status.EXECUTION_SUCCESS
        else:
            self.subtask_manager.hri.say("I've already received maximum assistance with placement.")
            return Status.EXECUTION_FAILURE

    def request_human_pointing(self):
        """Request human to point at the target location"""
        if self.apply_penalty("HUMAN_POINTING"):
            self.subtask_manager.hri.say("I'm having trouble determining where to place this object. Could you point to the correct location?")
            
            # Wait for pointing detection
            status, point = self.subtask_manager.vision.detect_pointing(timeout=30)
            
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("Thank you for pointing to the location.")
                return point
            else:
                self.subtask_manager.hri.say("I couldn't detect your pointing. I'll try my best.")
                return None
        else:
            self.subtask_manager.hri.say("I've already received maximum assistance with pointing.")
            return None

    def request_open_cabinet(self, door_number):
        """Request human to open cabinet door"""
        if self.apply_penalty("HUMAN_OPEN_CABINET"):
            self.subtask_manager.hri.say(f"I'm having trouble opening cabinet door {door_number}. Could you help open it?")
            
            # Wait for confirmation of door opening
            while not self.subtask_manager.hri.confirm("Have you opened the cabinet door?", timeout=30):
                self.subtask_manager.hri.say("Please open the cabinet door when ready.")
            
            self.subtask_manager.hri.say("Thank you for your assistance.")
            return Status.EXECUTION_SUCCESS
        else:
            self.subtask_manager.hri.say("I've already received maximum assistance with opening cabinets.")
            return Status.EXECUTION_FAILURE

    def handle_cereal_spill(self):
        """Handle cereal spilling scenario"""
        self.apply_penalty("SPILL_CEREAL")
        self.subtask_manager.hri.say("I apologize for spilling some cereal. I will be more careful.")
        # Could add cleanup behavior here

    def handle_leftover_cereal(self):
        """Handle leftover cereal in box scenario"""
        self.apply_penalty("LEAVE_CEREAL")
        self.subtask_manager.hri.say("I notice there is cereal left in the box. I should pour all of it.")
        
        # Additional attempt to pour remaining cereal
        return self.subtask_manager.manipulation.pour_remaining()

    def request_human_pour_cereal(self):
        """Request human assistance for pouring cereal"""
        if self.apply_penalty("HUMAN_POUR_CEREAL"):
            self.subtask_manager.hri.say("I'm having significant difficulty pouring the cereal. Could you please pour it for me?")
            
            # Wait for confirmation of pouring
            while not self.subtask_manager.hri.confirm("Have you poured the cereal?", timeout=30):
                self.subtask_manager.hri.say("Please pour the cereal when ready.")
            
            self.subtask_manager.hri.say("Thank you for your assistance with pouring.")
            return Status.EXECUTION_SUCCESS
        else:
            self.subtask_manager.hri.say("I've already received assistance with pouring.")
            return Status.EXECUTION_FAILURE

    def run(self):
        """State machine"""

        if self.current_state == StoringGroceriesTM.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            self.subtask_manager.hri.say("I am ready to start storing groceries.")
            self.current_state = StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_TABLE"]

        elif self.current_state == StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_TABLE"]:
            Logger.state(self, "Navigating to table")
            if self.navigate_to("table") == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I have arrived at the table.")
                self.current_state = StoringGroceriesTM.TASK_STATES["PERCEIVE_TABLE_OBJECTS"]
            else:
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    self.subtask_manager.hri.say("I couldn't reach the table after multiple attempts. Ending task.")
                    self.current_state = StoringGroceriesTM.TASK_STATES["END"]
                else:
                    self.subtask_manager.hri.say("Failed to reach the table. Trying again.")

        elif self.current_state == StoringGroceriesTM.TASK_STATES["PERCEIVE_TABLE_OBJECTS"]:
            Logger.state(self, "Perceiving objects on the table")
            self.subtask_manager.hri.say("I will now scan the table to identify groceries.")
            
            # Detect objects on the table
            status, objects = self.subtask_manager.vision.detect_objects(timeout=10)
            
            if status == Status.EXECUTION_SUCCESS and objects:
                self.objects_to_organize = objects
                self.subtask_manager.hri.say(f"I have detected {len(objects)} grocery items on the table.")
                
                # Check for cereal and container
                for obj in objects:
                    if "cereal" in obj['label'].lower():
                        self.cereal_found = True
                    if "container" in obj['label'].lower() or "bowl" in obj['label'].lower():
                        self.cereal_container_found = True
                
                # Go to the next object
                self.current_state = StoringGroceriesTM.TASK_STATES["PICK_OBJECT"]
            else:
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    self.subtask_manager.hri.say("I couldn't detect any groceries on the table. Moving on.")
                    self.current_state = StoringGroceriesTM.TASK_STATES["END"]
                else:
                    self.subtask_manager.hri.say("Failed to detect groceries. Trying again.")

        elif self.current_state == StoringGroceriesTM.TASK_STATES["PICK_OBJECT"]:
            if not self.objects_to_organize:
                # All objects have been processed
                if self.cereal_found and self.cereal_container_found:
                    self.subtask_manager.hri.say("I've stored all groceries from the table. Now I'll pour cereal.")
                    self.current_state = StoringGroceriesTM.TASK_STATES["OPEN_FIRST_CABINET"]
                else:
                    self.subtask_manager.hri.say("I've stored all groceries from the table.")
                    self.current_state = StoringGroceriesTM.TASK_STATES["END"]
                return
            
            # Get the next object
            self.current_object = self.objects_to_organize.pop(0)
            Logger.state(self, f"Picking up grocery item: {self.current_object['label']}")
            
            # Categorize the object
            self.current_object_category = self.categorize_object(self.current_object)
            self.subtask_manager.hri.say(f"I see a {self.current_object['label']}. It belongs to category: {self.current_object_category}")
            
            # Pick up the object
            status = self.subtask_manager.manipulation.pick(self.current_object['position'])
            
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"I have picked up the {self.current_object['label']}.")
                self.current_state = StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_CABINET"]
            else:
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    # Request human handover after multiple failures
                    if self.request_human_handover() == Status.EXECUTION_SUCCESS:
                        self.subtask_manager.hri.say(f"Now I have the {self.current_object['label']}.")
                        self.current_state = StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_CABINET"]
                    else:
                        self.subtask_manager.hri.say(f"Skipping this grocery item due to difficulties.")
                        # Reset attempts for next object
                        self.current_attempts = 0
                else:
                    self.subtask_manager.hri.say(f"Failed to pick up the {self.current_object['label']}. Trying again.")

        elif self.current_state == StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_CABINET"]:
            Logger.state(self, "Navigating to cabinet")
            if self.navigate_to("cabinet") == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I have arrived at the cabinet.")
                self.current_state = StoringGroceriesTM.TASK_STATES["PERCEIVE_CABINET_OBJECTS"]
            else:
                self.subtask_manager.hri.say("Failed to reach the cabinet. Will try to place the grocery item anyway.")
                self.current_state = StoringGroceriesTM.TASK_STATES["PERCEIVE_CABINET_OBJECTS"]

        elif self.current_state == StoringGroceriesTM.TASK_STATES["PERCEIVE_CABINET_OBJECTS"]:
            Logger.state(self, "Perceiving objects in cabinet")
            self.subtask_manager.hri.say("I will now scan the cabinet to understand its organization.")
            
            # Detect objects and layers in the cabinet
            status, cabinet_objects = self.subtask_manager.vision.detect_cabinet_layers(timeout=10)
            
            if status == Status.EXECUTION_SUCCESS:
                # Update cabinet layers knowledge
                for obj in cabinet_objects:
                    category = self.categorize_object(obj)
                    self.cabinet_layers[category] = obj['layer']
                
                self.current_state = StoringGroceriesTM.TASK_STATES["DETERMINE_PLACEMENT_LAYER"]
            else:
                self.subtask_manager.hri.say("I'm having trouble scanning the cabinet. I'll make my best guess for placement.")
                self.current_state = StoringGroceriesTM.TASK_STATES["DETERMINE_PLACEMENT_LAYER"]

        elif self.current_state == StoringGroceriesTM.TASK_STATES["DETERMINE_PLACEMENT_LAYER"]:
            Logger.state(self, "Determining placement layer")
            
            # Check if we already know where similar items are stored
            if self.current_object_category in self.cabinet_layers:
                target_layer = self.cabinet_layers[self.current_object_category]
                self.subtask_manager.hri.say(f"This {self.current_object['label']} should go on the {target_layer} layer with other {self.current_object_category} items.")
            else:
                # Determine based on predefined rules
                target_layer = self.determine_cabinet_layer(self.current_object_category)
                self.subtask_manager.hri.say(f"This {self.current_object['label']} belongs to {self.current_object_category} category and should go on the {target_layer} layer.")
                
                # Store this for future reference
                self.cabinet_layers[self.current_object_category] = target_layer
            
            self.current_state = StoringGroceriesTM.TASK_STATES["PLACE_OBJECT"]

        elif self.current_state == StoringGroceriesTM.TASK_STATES["PLACE_OBJECT"]:
            Logger.state(self, "Placing grocery in cabinet")
            
            # Get target layer position
            target_layer = self.cabinet_layers[self.current_object_category]
            target_position = {"layer": target_layer}
            
            # Check if there are similar objects already placed
            similar_objects = False
            for obj in self.cabinet_layers:
                if obj == self.current_object_category and obj != self.current_object['label']:
                    similar_objects = True
                    break
            
            # If having trouble, request pointing assistance before placement
            if self.current_attempts >= 1:
                target_point = self.request_human_pointing()
                if target_point:
                    target_position["point"] = target_point
            
            placement_status = Status.EXECUTION_FAILURE
            
            if similar_objects:
                self.subtask_manager.hri.say(f"I'll place this {self.current_object['label']} next to other {self.current_object_category} items.")
                # Enhanced placement next to similar objects
                placement_status = self.subtask_manager.manipulation.place_next_to_similar(target_position, self.current_object_category)
            else:
                self.subtask_manager.hri.say(f"I'll place this {self.current_object['label']} on the {target_layer} layer.")
                placement_status = self.subtask_manager.manipulation.place(target_position)
            
            if placement_status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"I have placed the {self.current_object['label']} successfully.")
                # Reset attempts counter for next object
                self.current_attempts = 0
                # Go back to table to pick up another object
                self.current_state = StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_TABLE"]
            else:
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    # Request human placement after multiple failures
                    if similar_objects:
                        self.request_human_placement(is_similar_objects=True)
                    else:
                        self.request_human_placement(is_similar_objects=False)
                    
                    # Reset attempts for next object
                    self.current_attempts = 0
                    # Go back to table for next object
                    self.current_state = StoringGroceriesTM.TASK_STATES["NAVIGATE_TO_TABLE"]
                else:
                    self.subtask_manager.hri.say(f"Having trouble placing the {self.current_object['label']}. Trying again.")

        elif self.current_state == StoringGroceriesTM.TASK_STATES["OPEN_FIRST_CABINET"]:
            Logger.state(self, "Opening first cabinet door")
            self.subtask_manager.hri.say("I will now open the first cabinet door to access the cereal storage area.")
            
            status = self.subtask_manager.manipulation.open_cabinet_door(1)
            
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I have opened the first cabinet door.")
                self.current_state = StoringGroceriesTM.TASK_STATES["OPEN_SECOND_CABINET"]
            else:
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    # Request human help after multiple attempts
                    self.request_open_cabinet(1)
                    self.current_state = StoringGroceriesTM.TASK_STATES["OPEN_SECOND_CABINET"]
                    self.current_attempts = 0
                else:
                    self.subtask_manager.hri.say("Failed to open the cabinet door. Trying again.")

        elif self.current_state == StoringGroceriesTM.TASK_STATES["OPEN_SECOND_CABINET"]:
            Logger.state(self, "Opening second cabinet door") 
            self.subtask_manager.hri.say("I will now open the second cabinet door.")
            
            status = self.subtask_manager.manipulation.open_cabinet_door(2)
            
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I have opened the second cabinet door.")
                self.current_state = StoringGroceriesTM.TASK_STATES["POUR_CEREAL"]
            else:
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    # Request human help after multiple attempts
                    self.request_open_cabinet(2)
                    self.current_state = StoringGroceriesTM.TASK_STATES["POUR_CEREAL"]
                    self.current_attempts = 0
                else:
                    self.subtask_manager.hri.say("Failed to open the second cabinet door. Trying again.")

        elif self.current_state == StoringGroceriesTM.TASK_STATES["POUR_CEREAL"]:
            Logger.state(self, "Pouring cereal into container")
            self.subtask_manager.hri.say("I will now pour cereal into the container.")
            
            # First pick up the cereal box
            cereal_object = None
            for obj in self.objects_to_organize:
                if "cereal" in obj['label'].lower():
                    cereal_object = obj
                    break
                    
            if cereal_object:
                status = self.subtask_manager.manipulation.pick(cereal_object['position'])
                if status != Status.EXECUTION_SUCCESS:
                    # Try one more time
                    status = self.subtask_manager.manipulation.pick(cereal_object['position'])
                    if status != Status.EXECUTION_SUCCESS:
                        # Request human handover for cereal
                        if self.request_human_handover() != Status.EXECUTION_SUCCESS:
                            self.subtask_manager.hri.say("I couldn't get the cereal box.")
                            self.current_state = StoringGroceriesTM.TASK_STATES["END"]
                            return
            else:
                self.subtask_manager.hri.say("I cannot find the cereal box.")
                self.current_state = StoringGroceriesTM.TASK_STATES["END"]
                return
                
            # Find the container
            container_object = None
            for obj in self.objects_to_organize:
                if "container" in obj['label'].lower() or "bowl" in obj['label'].lower():
                    container_object = obj
                    break
                    
            if container_object:
                # Pour the cereal
                self.subtask_manager.hri.say("Now pouring cereal into the container.")
                pour_status = self.subtask_manager.manipulation.pour(
                    source_object=cereal_object,
                    target_object=container_object
                )
                
                if pour_status == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say("I have poured the cereal into the container.")
                    
                    # Check if all cereal was poured
                    cereal_check = self.subtask_manager.vision.check_leftover_cereal()
                    if cereal_check > 0.1:  # More than 10% cereal left
                        # Handle leftover cereal
                        self.handle_leftover_cereal()
                else:
                    # Check if cereal was spilled
                    spill_check = self.subtask_manager.vision.check_cereal_spill()
                    if spill_check:
                        self.handle_cereal_spill()
                    
                    # Try pouring again
                    self.subtask_manager.hri.say("I'll try pouring again more carefully.")
                    pour_status = self.subtask_manager.manipulation.pour(
                        source_object=cereal_object,
                        target_object=container_object
                    )
                    
                    if pour_status != Status.EXECUTION_SUCCESS:
                        # Last resort - ask for human help
                        self.request_human_pour_cereal()
            else:
                self.subtask_manager.hri.say("I cannot find a suitable container for the cereal.")
            
            self.current_state = StoringGroceriesTM.TASK_STATES["END"]

        elif self.current_state == StoringGroceriesTM.TASK_STATES["END"]:
            Logger.state(self, "Ending task")
            self.subtask_manager.hri.say("I have completed storing all groceries.")
            self.subtask_manager.hri.say(f"Final score with penalties: {self.total_score}")
            self.running_task = False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = StoringGroceriesTM()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if _name_ == "_main_":
    main()