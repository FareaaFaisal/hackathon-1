---
id: 06-urdf-definition
title: 'URDF Definition'
sidebar_label: 'URDF Definition'
---

# Chapter 6: URDF Definition

## 6.1 Overview

The **Unified Robot Description Format (URDF)** is an XML-based file format used to describe the **physical structure of a robot**. It defines the robot's **links, joints, and sensors**, which are essential for simulation, motion planning, visualization, and state estimation in ROS 2.  

URDF provides a standardized method to communicate the robot’s geometry, kinematics, and dynamics to ROS 2 tools.

---

## 6.2 Key URDF Elements

### 6.2.1 `<robot>`

The root element of the URDF file. It contains all links, joints, and optionally sensors.

**Definition:** *Robot Element*: The top-level container of all structural and kinematic information in URDF.

---

### 6.2.2 `<link>`

Describes a rigid body of the robot. Each link includes:

- **Visual Properties**: Meshes, colors, materials for visualization.  
- **Collision Properties**: Geometry used for collision detection.  
- **Inertial Properties**: Mass, inertia matrix for dynamics calculations.

**Example:**

```xml
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
```
**Definition:** *Link*: A rigid segment of the robot, representing a physical part or body.

---

### 6.2.3 `<joint>`

Describes the connection between two links. Each joint defines:

- **Type:** e.g., revolute, prismatic, continuous, fixed.
- **Parent and Child Links:** Links connected by the joint.
- **Axis:** The axis of rotation or translation.
- **Limits:** Maximum and minimum allowed positions for actuated joints.

**Example:**

``` xml
<joint name="base_to_right_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="right_wheel"/>
  <axis xyz="0 1 0"/>
</joint>
```

**Definition:** *Joint*: The kinematic connection between two links, defining relative motion and constraints.

---

### 6.2.4 `<sensor>`

Describes a sensor attached to a link. Each sensor can include:

- Type: e.g., camera, LiDAR.
- Position and Orientation relative to the parent link.
- Visual/Collision Representation: Optional geometry for simulation.
- Update Rate: How often the sensor publishes data.

***Definition:*** *Sensor*: A simulated or physical device providing environmental or proprioceptive data to the robot.

---

## 6.3 Example URDF

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
    <!-- Additional link properties here -->
  </link>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

This URDF defines a simple robot with a base link and a wheel connected by a continuous joint. Additional links, joints, and sensors can be added similarly.

## 6.4 Summary

- URDF is essential for describing a robot’s geometry, kinematics, and dynamics.
- Links define rigid bodies with visual, collision, and inertial properties.
- Joints describe how links are connected and move relative to each other.
- Sensors provide environmental or proprioceptive feedback.
- URDF files integrate with ROS 2 tools for visualization, motion planning, and simulation.

## 6.5 Learning Outcomes

After completing this chapter, students will be able to:

- Define the purpose and structure of a URDF file.
- Describe links, joints, and sensors in URDF.
- Create a simple URDF to define the physical structure of a humanoid robot.
- Understand how URDF integrates with ROS 2 motion planning and visualization tools.

## References

[1] W. Meeussen et al., “URDF: Unified Robot Description Format,” ROS Documentation, 2012.

[2] Siciliano, B., et al., Springer Handbook of Robotics, 2nd ed., Springer, 2016.

[3] ROS 2 Documentation, “Robot Description with URDF,” https://docs.ros.org/en/humble/Tutorials/URDF/URDF.html.

[4] Quigley, M., et al., “ROS: An Open-Source Robot Operating System,” ICRA Workshop on Open Source Software, 2009.


In the next chapter, we will learn how to use launch files to start multiple ROS 2 nodes at once.
