---
id: 05-humanoid-communication
title: 'Humanoid Communication'
sidebar_label: 'Humanoid Communication'
---

# Chapter 5: Humanoid Communication

## 5.1 Overview

Effective communication between a humanoid robot's components is crucial for coordinated behavior. In ROS 2, communication occurs through **Topics, Services, and Actions**, enabling different nodes to exchange information in real-time. This chapter explores the communication patterns necessary for humanoid tasks such as **walking, grasping, and perception**.

---

## 5.2 Joint State Communication

The **joint state** represents the positions, velocities, and efforts of the robot's actuators. Publishing this information ensures that motion planners, state estimators, and visualization tools operate with accurate data.

**Key Details**:

- **Topic:** `/joint_states`  
- **Message Type:** `sensor_msgs/msg/JointState`  
- **Publisher:** Robot's motor controllers  
- **Subscribers:** State estimator, motion planner, and RViz  

**Definition:** *Joint State*: A ROS 2 message describing the current position, velocity, and effort of all robot joints.  

This communication pattern allows components to monitor and respond to the robot's physical configuration in real-time.

---

## 5.3 Motion Planning

Motion planning involves generating **collision-free trajectories** for the robot’s limbs. We use **MoveIt 2**, a ROS 2 motion planning framework, which provides services and actions for planning and executing movements.

**Key Details**:

- **Action:** `/move_group`  
- **Action Type:** `moveit_msgs/action/MoveGroup`  
- **Action Server:** MoveIt 2 `move_group` node  
- **Action Client:** Any node requesting a motion plan  

**Definition:** *Motion Planning*: The computational process of determining a path from the robot's current state to a desired state while avoiding obstacles.  

Through this communication pattern, humanoid robots can plan complex motions like walking, reaching, or grasping objects.

---

## 5.4 Perception Communication

The **perception system** processes sensor data to understand the environment. It integrates information from cameras, LiDAR, and other sensors to enable tasks like object detection, mapping, and human-robot interaction.

**Key Details**:

- **Topic:** `/camera/image_raw`  
- **Message Type:** `sensor_msgs/msg/Image`  
- **Publisher:** Robot’s camera driver  
- **Subscribers:** Object detection node, visual SLAM node, human-robot interaction node  

**Definition:** *Perception*: The process of interpreting sensor data to build an understanding of the robot's environment.  

This communication ensures that all components dependent on visual or depth information receive real-time updates for decision-making and control.

---

## 5.5 Summary

- Humanoid robots rely on ROS 2 **Topics, Services, and Actions** for communication.  
- **Joint state communication** provides actuator feedback to planners and visualization tools.  
- **Motion planning actions** allow nodes to request collision-free trajectories using MoveIt 2.  
- **Perception topics** distribute sensor information to multiple consumers for environment understanding.  
- These patterns form the foundation for coordinated humanoid behavior and complex task execution.

---

## 5.6 Learning Outcomes

After completing this chapter, students will be able to:

1. Explain the role of Topics, Services, and Actions in humanoid robot communication.  
2. Describe how joint states are published and consumed by multiple components.  
3. Understand motion planning communication using MoveIt 2 actions.  
4. Identify perception topics and how sensor data is shared across nodes.  

---

## References

[1] S. Chitta et al., “MoveIt 2: An Open Source Robotics Framework for Motion Planning,” *IEEE Robotics and Automation Letters*, vol. 3, no. 3, pp. 2200–2207, 2018.  
[2] ROS 2 Documentation, “Topics, Services, and Actions,” *https://docs.ros.org/en/humble/Concepts/About-Communication.html*.  
[3] Quigley, M., et al., “ROS: An Open-Source Robot Operating System,” *ICRA Workshop on Open Source Software*, 2009.  
[4] Siciliano, B., et al., *Springer Handbook of Robotics*, 2nd ed., Springer, 2016.
