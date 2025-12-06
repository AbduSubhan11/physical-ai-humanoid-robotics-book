---
title: URDF for Humanoids
description: Defining robotic structures with URDF.
slug: urdf-for-humanoids
---

# URDF for Humanoids: Describing Robot Kinematics and Dynamics

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe the physical characteristics of a robot. It allows you to model the robot's kinematics (how its parts are connected and move) and dynamics (its mass, inertia, and collision properties). For humanoid robots, which have complex articulated structures resembling humans, URDF is indispensable for simulation, visualization, and motion planning.

## What is URDF?

URDF represents a robot as a collection of **links** and **joints**:

*   **Links:** These are the rigid bodies of the robot, such as a torso, an arm segment, or a foot. Each link has properties like its visual appearance, collision geometry, mass, and inertia.
*   **Joints:** These define the connections between links and specify how they can move relative to each other. Joints can be revolute (rotational), prismatic (linear), fixed, continuous, and more.

For humanoid robots, the URDF structure often mirrors the human skeletal system, with links representing bones and joints representing articulations like knees, elbows, and hips.

## Key Elements of a URDF File

A typical URDF file is structured around the `<robot>` tag, containing multiple `<link>` and `<joint>` definitions.

### `<link>` Element

Each `<link>` defines a rigid body. Key sub-elements include:

*   **`<visual>`:** Describes the visual appearance of the link. This includes the geometry (e.g., `box`, `cylinder`, `sphere`, or `mesh` for complex 3D models), its origin (offset from the link's own frame), and material properties (color or texture).
*   **`<collision>`:** Defines the collision geometry of the link, used for physics simulation and collision detection. It often mirrors the visual geometry but can be simplified for computational efficiency.
*   **`<inertial>`:** Specifies the physical properties of the link, including its mass, center of mass (origin), and inertia tensor. These are crucial for accurate physics simulations.

Example Link (simplified):
```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.2 0.4 0.6" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.4 0.6" />
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0" />
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
</link>
```

### `<joint>` Element

Each `<joint>` defines a connection between two links:

*   **`name`:** A unique identifier for the joint.
*   **`type`:** Specifies the type of joint (e.g., `revolute`, `prismatic`, `fixed`, `continuous`).
*   **`<parent link="..."/>`:** Specifies the name of the parent link.
*   **`<child link="..."/>`:** Specifies the name of the child link.
*   **`<origin>`:** Defines the transformation (position and orientation) of the joint frame relative to the parent link's frame.
*   **`<axis>`:** For revolute or prismatic joints, this defines the axis of rotation or translation.
*   **`<limit>`:** For revolute or prismatic joints, defines the upper and lower limits of movement, as well as velocity and effort limits.

Example Joint (simplified):
```xml
<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.4" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

## XACRO: Enhancing URDF Flexibility

While URDF is powerful, it can become verbose for complex robots, especially humanoids with many similar components (e.g., fingers, toes). **XACRO (XML Macros)** is an XML macro language that extends URDF, allowing for:

*   **Macros:** Define reusable blocks of URDF XML, reducing repetition.
*   **Parameters:** Use variables to easily adjust dimensions, masses, or other properties.
*   **Mathematical Expressions:** Perform calculations within the URDF, such as defining joint limits based on other parameters.

XACRO files are processed into standard URDF before being used by ROS 2 tools.

## Applications of URDF for Humanoid Robots

*   **Visualization:** URDF files are used by tools like RViz to visualize the robot's model, joint states, and sensor data in a 3D environment.
*   **Simulation:** Physics engines like Gazebo and Unity (with appropriate plugins) use URDF to simulate the robot's dynamics, interactions with the environment, and sensor behavior.
*   **Motion Planning:** Libraries like MoveIt (ROS 1) and MoveIt 2 (ROS 2) parse URDF to understand the robot's kinematic chain, allowing for complex motion planning (e.g., inverse kinematics for reaching tasks).
*   **Control:** Controllers often use the robot description to understand joint limits and current configurations for safe and effective operation.
*   **Collision Detection:** The collision geometry defined in URDF is used to detect potential collisions between robot parts and with the environment.

## Designing URDF for Humanoids

Designing a URDF for a humanoid robot involves careful consideration of:

*   **Degrees of Freedom (DoF):** Humanoids have many DoF, requiring a precise definition of each joint.
*   **Coordinate Frames:** Consistent definition of coordinate frames for each link and joint is critical.
*   **Mass and Inertia:** Accurate physical properties are essential for realistic simulation.
*   **Mesh Models:** Using well-designed 3D mesh models (e.g., STL, DAE) for visual and collision geometries enhances realism and accuracy.

URDF, especially when combined with XACRO, provides a robust framework for describing the intricate mechanical structure of humanoid robots, enabling a wide range of applications from realistic simulation to advanced control and planning. Understanding and correctly implementing URDF is a foundational skill for anyone working with physical AI and humanoid robotics.
