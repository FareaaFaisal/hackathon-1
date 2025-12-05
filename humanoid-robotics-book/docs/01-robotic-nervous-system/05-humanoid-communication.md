---
id: 05-humanoid-communication
title: 'Humanoid Communication'
sidebar_label: 'Humanoid Communication'
---

# Chapter 5: Humanoid Communication

Effective communication between a humanoid robot's various components is crucial for its operation. In this chapter, we will explore the communication patterns used in our humanoid robot, focusing on the specific topics, services, and actions required for tasks such as walking, grasping, and perception.

### Joint State Communication:

The state of the robot's joints (e.g., position, velocity, and effort) is published to the `/joint_states` topic. This information is used by various components, including the robot's state estimator, motion planner, and visualization tools.

-   **Topic**: `/joint_states`
-   **Message Type**: `sensor_msgs/msg/JointState`
-   **Publisher**: The robot's motor controllers.
-   **Subscribers**: The robot's state estimator, motion planner, and RViz.

### Motion Planning:

Motion planning is the process of generating a collision-free path for the robot's limbs. We will use the MoveIt 2 motion planning framework, which provides a set of services and actions for motion planning.

-   **Action**: `/move_group`
-   **Action Type**: `moveit_msgs/action/MoveGroup`
-   **Action Server**: The MoveIt 2 `move_group` node.
-   **Action Client**: Any node that needs to request a motion plan.

### Perception:

The robot's perception system is responsible for processing sensor data to understand the environment. This includes data from cameras, LiDAR, and other sensors.

-   **Topic**: `/camera/image_raw`
-   **Message Type**: `sensor_msgs/msg/Image`
-   **Publisher**: The robot's camera driver.
-   **Subscribers**: The object detection node, the visual SLAM node, and the human-robot interaction node.

In the next chapter, we will learn how to define the physical structure of our robot using the URDF file format.
