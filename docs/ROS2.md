# ROS2 @Home Guide

## Main commands
### Build
From the root of the workspace (home2) with ROS2 installed.

Build all packages (this should be run after updating a script, dependencies, packages, etc):
``` bash
colcon build
```

Build specific package/s:
``` bash
colcon build --packages-select <package_name1> <package_name2> ...
```
``` bash
colcon build --packages-select frida_interfaces vision_general
```

Source the workspace (this is needed to run the nodes):
``` bash
source install/setup.bash
```

### Run
Run a node:
``` bash
ros2 run <package_name> <node_name>
```
``` bash
ros2 run vision_general example_node
```

Run a launch file (this is a file that can run multiple nodes):
``` bash
ros2 launch <package_name> <launch_file>
```
``` bash
ros2 launch vision_general example_launch.py
```

### Debug
Check current topics:
``` bash
ros2 topic list
```

For build issues, you can clean the workspace and rebuild:
``` bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## General Node Structure (Python)
In general, a node is a script that will run and interact with the ROS2 system through topics, services, actions, etc.
A `topic` is a channel where nodes can send and receive messages. 

``` python
#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionServer

# Topics
SUBSCRIBER_TOPIC = "/example/subscriber_topic"
PUBLISHER_TOPIC = "/example/publisher_topic"

# Node
class ExampleNode(Node):
    def __init__(self):
        """Initial setup for the node"""

        # Initialize Node with name
        super().__init__("example_node")

        # Add publishers, subscribers, services, action services, etc.
        self.example_publisher = self.create_publisher(String, PUBLISHER_TOPIC, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  

        # Use logger to print messages (this can also be .warn() or .error())
        self.get_logger().info("Example node ready.")

    # Define callbacks
    def timer_callback(self):
        pass

# Main function
def main(args=None):
    # Initialize node
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        # Spin the node (keep it running)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Run main function
if __name__ == "__main__":
    main()

```
Check the complete node in [example_node.py](../vision/vision_general/scripts/example_node.py)

## Subscribers
A subscriber will listen to a topic and receive messages from it. 
It should have a callback function that will be called every time a message is received. In this case the type of the message is `String`, which sould be imported from `std_msgs.msg`.

``` python
    def __init__(self):
        super().__init__("example_node")

        # Create Subscriber
        self.example_subscriber = self.create_subscription(String, "/example/subscriber_topic", self.subscriber_callback, 10)

    def subscriber_callback(self, msg):
        # Print message data each time a message is received
        self.get_logger().info("I heard: " + msg.data)
```

To test a subscriber, you can run the node in one terminal and publish messages in another terminal:
``` bash
ros2 run vision_general example_node
```

``` bash
ros2 topic pub /example/subscriber_topic std_msgs/msg/String "data: hi"
```

## Publishers and Timers
A publisher is a node that sends messages to a topic. After creating the publisher, you can create a message and publish it. 
If you want to send messages periodically, you can use a timer that calls a function every x seconds.
This example also publishes a message of type `String`.

``` python
    def __init__(self):
        super().__init__("example_node")

        # Create Publisher
        self.example_publisher = self.create_publisher(String, "/example/publisher_topic", 10)

        # Create Timer: this will call a function called timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)  

    def timer_callback(self):
        # Create message and set data
        msg = String()
        msg.data = "Hello World"

        # Publish message
        self.example_publisher.publish(msg)
```

To test a publisher, you can run the node in one terminal and listen to the topic in another terminal:
``` bash
ros2 run vision_general example_node
```

``` bash
ros2 topic echo /example/publisher_topic
```

## Services
A service is a node that provides a service to other nodes. This means that it receives a request and should return a response.
For this, it should have a callback function that will be called when the service is requested.

In this example, we are using a srv file called `SetBool` that has the following structure:

``` yaml
bool data
---
bool success 
string message
```
This meanas that the request will have a boolean data field, and the response will have a boolean success field and a string message field. This are separated by `---`. This standard service should be imported from `std_srvs.srv`.

``` python
    def __init__(self):
        super().__init__("example_node")

        # Create Service
        self.example_service = self.create_service(SetBool, "/example/service_topic", self.service_callback)

    def service_callback(self, request, response):
        # Get the request data
        req = request.data

        # Print
        if req:
            self.get_logger().info("Service called with request: True")
        else:
            self.get_logger().info("Service called with request: False")

        # Set the response data
        response.message = "Result msg"
        response.success = True

        return response
```

To test a service, you can run the node in one terminal and call the service in another terminal.
```bash
ros2 service call <topic_name> <service_type> "<request_data>"
```
Example:

``` bash
ros2 run vision_general example_node
```

``` bash
ros2 service call /example/service_topic std_srvs/srv/SetBool "data: true"
```

## Action services
An action service is a more complex service that allows for feedback and goal handling.
This also receives a request and should return a response, but it can also send feedback during the process. The ActionServer should be imported from `rclpy.action`.
In this example we are using an action file called `DetectPerson` that should be imported from `frida_interfaces` and has the following structure:

``` yaml
bool request
---
bool success
---
string feedback
``` 


``` python
    def __init__(self):
        super().__init__("example_node")

        # Create Action Service
        self.example_action_server = ActionServer(self, DetectPerson, "/example/action_service_topic", self.action_callback)

    def action_callback(self, goal_handle):
        # Get the request data
        self.goal_handle = goal_handle
        request = goal_handle.request
        self.get_logger().info(f"Goal received: {request}")

        # Simulate a process and send feedback (It just counts from 1-5)
        for i in range(1,6):
            # Check if the request was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return DetectPerson.Result()

            # Create feedback message
            feedback_msg = DetectPerson.Feedback()
            feedback_msg.feedback = str(i)

            # Send feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: {i}")
            time.sleep(1)

        # After finishing a process, send the result
        result = DetectPerson.Result()
        result.success = True
        goal_handle.succeed()
        return result
```

To test an action service, you can run the node in one terminal and call the action service in another terminal.
```bash
ros2 action send_goal <action_name> <action_type> "<goal_data>" --feedback
```
Example:

``` bash
ros2 run vision_general example_node
```

``` bash
ros2 action send_goal /example/action_service_topic frida_interfaces/action/DetectPerson "request: true" --feedback
```

## Interfaces
Interfaces are files that define the structure of messages, services, and actions. They are defined in `.msg`, `.srv`, and `.action` files respectively.
For this repo, the interfaces are located in the `frida_interfaces` package according to each area.

### Message
A message is a structure that can be sent through a topic. It can have multiple fields of different types.

Example:
In `frida_interfaces/vision/msg/Person.msg`:

``` yaml
# Example message for the coordinates of a person
string name
int64 x
int64 y
```

A message can also include an array of other messages like in `frida_interfaces/vision/msg/PersonList.msg`:

``` yaml
# Example message for an array of people
frida_interfaces/Person[] list
```

### Service
A service is a structure that defines a request and a response. It can have multiple fields of different types.

Example:
In `frida_interfaces/vision/srv/FindSeat.srv`:

``` yaml
bool request 
---
float32 angle
bool success
```

### Action
An action is a structure that defines a goal, feedback, and result. It can have multiple fields of different types.

Example:
`ExampleDetectObject.action`:

``` yaml
# Request
string objectName
---
# Result
bool success
int32 x
int32 y
---
# Feedback
string feedback
string currentObject
```

