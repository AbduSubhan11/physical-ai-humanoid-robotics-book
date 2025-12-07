---
title: rclpy Example Controllers
description: Implementing robotic control using rclpy.
slug: rclpy-example-controllers
---

# rclpy Example Controllers: Bringing Robots to Life with Python

`rclpy` is the Python client library for ROS 2, providing a convenient and powerful way to develop ROS 2 applications using the Python programming language. It allows developers to create nodes, publish to topics, subscribe to topics, call services, and implement service servers, among other functionalities. This chapter will explore how to use `rclpy` to build basic robotic controllers, focusing on practical examples that demonstrate fundamental ROS 2 concepts.

## Setting Up Your Python Environment for `rclpy`

Before diving into controller development, ensure your environment is correctly set up:

1. **ROS 2 Installation:** Make sure you have a working ROS 2 installation (e.g., Foxy, Galactic, Humble, Iron).
2. **Python 3:** `rclpy` requires Python 3.
3. **ROS 2 Python Packages:** Ensure the necessary ROS 2 Python packages are installed, typically handled during the main ROS 2 installation. You might need to source your ROS 2 environment.

## Basic Controller Structure in `rclpy`

A typical `rclpy` node that functions as a controller will generally involve:

1. **Import `rclpy`:** Start by importing the `rclpy` library.
2. **Create a Node:** Instantiate a new `Node` object.
3. **Create Publishers/Subscribers:** Set up communication channels. For a controller, this often means subscribing to sensor data (e.g., odometry) and publishing control commands (e.g., `Twist` messages for velocity control).
4. **Callback Functions:** Define functions that are executed when new messages arrive on subscribed topics.
5. **Spinning the Node:** Use `rclpy.spin()` to keep the node alive and process callbacks.

## Example: A Simple Differential Drive Robot Controller

Let's consider a common scenario: controlling a differential drive robot. This type of robot typically receives velocity commands (linear and angular) and publishes its odometry (position and orientation). A simple controller might subscribe to joystick commands and publish corresponding `Twist` messages.

### 1. `teleop_publisher.py` (Simulated Joystick/Command Publisher)

This node will simulate user input, publishing `Twist` messages to `/cmd_vel`.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TeleopPublisher(Node):

    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)  # Publish every 0.5 seconds
        self.get_logger().info('Teleop Publisher Node started')

        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.direction = 0  # 0: forward, 1: turn left, 2: turn right, 3: backward

    def publish_cmd_vel(self):
        twist_msg = Twist()

        if self.direction == 0:
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        elif self.direction == 1:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_speed
            self.get_logger().info('Turning left')
        elif self.direction == 2:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -self.angular_speed
            self.get_logger().info('Turning right')
        elif self.direction == 3:
            twist_msg.linear.x = -self.linear_speed
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving backward')

        self.publisher_.publish(twist_msg)

        # Cycle through directions for demonstration
        self.direction = (self.direction + 1) % 4

def main(args=None):
    rclpy.init(args=args)
    teleop_publisher = TeleopPublisher()
    try:
        rclpy.spin(teleop_publisher)
    except KeyboardInterrupt:
        pass
    teleop_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
