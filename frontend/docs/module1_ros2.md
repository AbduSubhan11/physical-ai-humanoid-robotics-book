# Module 1 — The Robotic Nervous System (ROS 2)

The Robotic Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. ROS 2 is the latest iteration, designed with improved real-time capabilities, security, and multi-robot system support.

## ROS 2 Nodes

At the core of ROS 2's architecture are **nodes**. A node is an executable process that performs computation. Nodes are typically designed to do one thing well, following the Unix philosophy. For example, one node might control a motor, another might read data from a sensor, and yet another might perform path planning.

Nodes communicate with each other using the ROS 2 communication mechanisms, forming a distributed system. This modularity allows for easy development, testing, and debugging of individual components.

Each node in a ROS 2 system has a unique name, which can be specified when launching the node or overridden via remapping rules. This naming convention helps in organizing and identifying different components in a complex robotic system.

**Example: Creating a simple ROS 2 Node in Python**

To create a node, you first need to initialize the ROS 2 client library for Python (`rclpy`). Then, you can create an instance of a `Node` and spin it, allowing it to perform its work (e.g., publish messages, subscribe to topics, provide services).


```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: \"%s\"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


## Topics & Services

ROS 2 provides several mechanisms for inter-node communication. The two most fundamental are **topics** and **services**.

### Topics

**Topics** are used for asynchronous, many-to-many communication. A node can "publish" messages to a topic, and any number of other nodes can "subscribe" to that topic to receive those messages. This is a broadcast-style communication where publishers don't know or care who the subscribers are, and subscribers don't know or care who the publishers are.

Topics are ideal for streaming data, such as sensor readings (e.g., camera images, LiDAR scans), joint states, or odometry information, where continuous updates are needed. Each topic has a name and a message type, which defines the structure of the data being transmitted.

**Example: Publisher and Subscriber in Python**

(See the `MinimalPublisher` example above for a publisher. Here's a subscriber.)


```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self):
        self.get_logger().info('I heard: \"%s\"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


### Services

**Services** are used for synchronous, request-response style communication. A "service client" sends a request to a "service server," which then processes the request and sends back a response. This is a one-to-one communication pattern, commonly used for actions that require a direct response, such as triggering a specific robot behavior, querying a database, or performing a calculation.

Services are defined by a request message type and a response message type.

**Example: Service Server and Client in Python**

**Service Server:**


```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


**Service Client:**


```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


## rclpy example controllers

`rclpy` is the Python client library for ROS 2. It provides the necessary APIs to write ROS 2 nodes in Python, allowing developers to interact with topics, services, parameters, and other ROS 2 constructs.

Developing controllers in ROS 2 often involves:
1.  **Subscribing to sensor feedback:** Reading data from IMUs, encoders, cameras, etc., to understand the robot's current state.
2.  **Implementing control logic:** Applying algorithms (e.g., PID, state machines, inverse kinematics) to determine desired actions.
3.  **Publishing command messages:** Sending commands to actuators (e.g., motor speeds, joint positions).
4.  **Providing/Using services:** For higher-level commands or queries.

Let's consider a basic example of an `rclpy` controller that makes a robot (simulated or real) move forward and then turn. This would involve a simple state machine.

**Simplified Differential Drive Robot Controller Example (Conceptual)**

This example assumes the existence of:
*   A topic `/cmd_vel` (of type `geometry_msgs/Twist`) to send linear and angular velocity commands.
*   A topic `/odom` (of type `nav_msgs/Odometry`) to receive robot pose and velocity feedback. (For simplicity, we'll omit odometry feedback for this conceptual example).


```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz control loop
        self.state = 'forward'
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def control_loop(self):
        twist_msg = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        if self.state == 'forward':
            twist_msg.linear.x = 0.2  # Move forward at 0.2 m/s
            twist_msg.angular.z = 0.0
            if elapsed_time > 3.0: # Move forward for 3 seconds
                self.get_logger().info('Transitioning to turn state.')
                self.state = 'turn'
                self.start_time = current_time # Reset timer for the next state
        elif self.state == 'turn':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5 # Turn at 0.5 rad/s
            if elapsed_time > 2.0: # Turn for 2 seconds
                self.get_logger().info('Transitioning to stop state.')
                self.state = 'stop'
                self.start_time = current_time
        elif self.state == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Robot stopped.')
            # In a real scenario, you might want to shutdown or wait for a new command
            # For this example, we'll just keep publishing zeros.

        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


## URDF for Humanoids

The **Unified Robot Description Format (URDF)** is an XML format used in ROS to describe all aspects of a robot. This includes its kinematic and dynamic properties, visual appearance, and collision properties. For humanoid robots, URDF is crucial because it allows for a precise definition of a complex, multi-articulated body.

A URDF file defines a robot as a collection of **links** and **joints**:
*   **Links:** Represent the rigid bodies of the robot (e.g., torso, upper arm, forearm, hand). Each link can have visual properties (how it looks), inertial properties (mass, inertia tensor), and collision properties (how it interacts with other objects).
*   **Joints:** Connect two links and define their relative motion. Joints can be of various types (revolute, prismatic, fixed, continuous) and have properties like limits, dynamics (friction, damping), and safety limits.

For humanoid robots, URDF files can become very large and complex due to the high number of degrees of freedom. Tools like Xacro (XML Macros) are often used in conjunction with URDF to simplify the creation and maintenance of these complex robot descriptions by allowing the use of variables, math, and conditional statements.

**Key elements in a Humanoid URDF:**

*   **`robot` tag:** The root element, containing all links and joints.
*   **`link` tag:** Defines a rigid body.
    *   `visual`: Describes the visual model (e.g., mesh file, color) and its origin (offset from the link's reference frame).
    *   `collision`: Describes the collision model (e.g., primitive shape, mesh) and its origin.
    *   `inertial`: Defines mass and inertia matrix, crucial for physics simulation.
*   **`joint` tag:** Connects two links.
    *   `parent` and `child`: Specify the two links being connected.
    *   `type`: e.g., `revolute`, `continuous` (for rotating joints), `prismatic` (for sliding joints), `fixed` (for rigid connections).
    *   `origin`: Defines the joint's position and orientation relative to the parent link.
    *   `axis`: For revolute/prismatic joints, specifies the axis of rotation or translation.
    *   `limit`: Defines the upper and lower limits for joint position, velocity, and effort.
*   **`transmission` tag:** (Often in a separate Xacro file or included) describes how actuators are connected to joints. This is essential for interfacing with hardware controllers.
*   **`gazebo` tag:** (Also often in separate files) provides Gazebo-specific properties, such as material colors for simulation, sensor plugins, and more accurate physics properties.

**Simplified URDF Snippet for a Humanoid Arm Joint (Conceptual)**


```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">

  <link name="base_link"/>

  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="10.0" velocity="1.0"/>
  </joint>

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>

</robot>
```