---
title: Topics & Services
description: Exploring communication mechanisms in ROS 2.
slug: topics-services
---

# ROS 2 Communication: Topics and Services

In ROS 2, effective communication between nodes is crucial for building complex robotic systems. The framework provides several primary mechanisms for nodes to interact, with **topics** and **services** being two of the most fundamental. These mechanisms cater to different communication patterns, allowing developers to design robust and efficient inter-node data exchange.

## Topics: Asynchronous Data Streaming

**Topics** are the most common communication mechanism in ROS 2, designed for asynchronous, one-to-many data streaming. They are ideal for broadcasting continuous or periodic data from one node to multiple interested nodes.

### How Topics Work:

1.  **Publisher:** A node that produces data publishes messages to a named topic.
2.  **Subscriber:** One or more nodes that are interested in that data subscribe to the same topic.
3.  **Message Type:** Every topic has a defined message type (e.g., `sensor_msgs/msg/LaserScan`, `std_msgs/msg/String`). This ensures that all data exchanged on that topic conforms to a specific structure, preventing data interpretation issues.
4.  **Decoupling:** Publishers and subscribers are decoupled. A publisher doesn't need to know which subscribers are listening, and subscribers don't need to know which publisher is sending data. This flexibility allows for dynamic system configurations.
5.  **Quality of Service (QoS):** ROS 2 introduces QoS settings, which allow developers to configure the reliability, durability, and other characteristics of topic communication. This is critical for different types of data, such as sensor readings (often best effort) versus critical command signals (often reliable).

### When to Use Topics:

*   Streaming sensor data (e.g., camera images, LiDAR scans, IMU readings).
*   Broadcasting robot state information (e.g., odometry, joint states).
*   Sending continuous control commands (e.g., velocity commands to motors).
*   Any scenario where data needs to be sent repeatedly to potentially multiple receivers.

## Services: Synchronous Request-Response

**Services** provide a synchronous, one-to-one communication mechanism, designed for request-response interactions. They are suitable for tasks that require an immediate answer or a single, atomic operation.

### How Services Work:

1.  **Service Server:** A node that offers a specific functionality acts as a service server. It waits for incoming requests.
2.  **Service Client:** A node that needs that functionality acts as a service client. It sends a request to the server.
3.  **Service Type:** Similar to topics, services have a defined service type, which specifies the structure of both the request and the response messages.
4.  **Blocking Call:** When a client makes a service call, it typically blocks (waits) until it receives a response from the server or a timeout occurs. This synchronous nature ensures that the client receives immediate feedback on the operation.
5.  **One-to-One:** A service call is always initiated by a single client and handled by a single server (though multiple clients can call the same server, and multiple servers can offer the same service).

### When to Use Services:

*   Triggering an action that returns a result (e.g., `calculate_path` and receive the path).
*   Querying for current state information (e.g., `get_robot_pose`).
*   Performing an infrequent, atomic task (e.g., `set_gripper_position`).
*   Any scenario where a client requires an immediate response to a specific request.

## Topics vs. Services: A Comparison

| Feature           | Topics                               | Services                                 |
| :---------------- | :----------------------------------- | :--------------------------------------- |
| Communication     | Asynchronous, one-to-many            | Synchronous, one-to-one                  |
| Data Flow         | Continuous stream                    | Single request-response cycle            |
| Use Case          | Sensor data, robot state, commands   | Atomic actions, queries, configuration   |
| Blocking          | Non-blocking                         | Blocking (client waits for response)     |
| Data Persistence  | No inherent persistence              | Request and response are distinct        |

Choosing between topics and services depends on the specific communication needs of your application. Topics are excellent for broadcasting dynamic data, while services are perfect for single-shot, reliable operations. Understanding both mechanisms is key to designing well-structured and efficient ROS 2 robotic applications. The next chapter will explore `rclpy` to demonstrate practical implementation of these concepts.