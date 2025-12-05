---
id: 06-urdf-definition
title: 'URDF Definition'
sidebar_label: 'URDF Definition'
---

# Chapter 6: URDF Definition

The **Unified Robot Description Format (URDF)** is an XML file format used to describe the physical structure of a robot. This includes the robot's links, joints, and sensors. The URDF file is used by various ROS 2 tools, including the robot's state estimator, motion planner, and visualization tools.

### Key URDF Elements:

-   **`<robot>`**: The root element of the URDF file.
-   **`<link>`**: Describes a rigid body of the robot. This includes its visual properties (e.g., mesh, material), collision properties (e.g., geometry, origin), and inertial properties (e.g., mass, inertia).
-   **`<joint>`**: Describes the connection between two links. This includes the joint type (e.g., revolute, prismatic, fixed), the parent and child links, and the joint limits (e.g., upper and lower position limits).
-   **`<sensor>`**: Describes a sensor attached to a link. This includes the sensor type (e.g., camera, LiDAR), the sensor's visual and collision properties, and the sensor's update rate.

### Example URDF:

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <link name="right_wheel">
    ...
  </link>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

In the next chapter, we will learn how to use launch files to start multiple ROS 2 nodes at once.
