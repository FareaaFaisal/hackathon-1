---
sidebar_position: 5
title: Sensor Simulation
---

## Chapter 05: 

Sensors are critical for humanoid robots to perceive their environment. In simulation, we can **emulate LiDAR, RGB cameras, Depth cameras, and IMUs** to test perception algorithms before deployment. Using Gazebo and ROS 2, simulated sensors publish data to **ROS 2 topics**, allowing developers to integrate perception pipelines and verify functionality in a safe environment.

---

## 5.1 LiDAR Simulation

**LiDAR (Light Detection and Ranging)** sensors provide 2D or 3D point clouds representing the environment. In Gazebo, LiDAR is defined as a sensor plugin in the robot's URDF or SDF file:

```xml
<sensor name="lidar_sensor" type="ray">
  <pose>0 0 1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.5708</min_angle>
        <max_angle>1.5708</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

- Publishes sensor_msgs/msg/LaserScan data to a ROS 2 topic (e.g., /lidar_scan).
- Used for obstacle detection, mapping, and navigation.

> Tip: Adjust the number of samples and scan resolution for performance vs. fidelity trade-offs.

---

## 5.2 RGB Camera Simulation

RGB cameras simulate standard color cameras used for vision tasks. In Gazebo:

```bash
<sensor name="rgb_camera" type="camera">
  <camera>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>50.0</far>
    </clip>
  </camera>
</sensor>
```

- Publishes sensor_msgs/msg/Image on a ROS 2 topic (e.g., /camera/rgb/image_raw).
- Commonly used for object detection, visual SLAM, and human-robot interaction.

> Tip: Ensure the camera frame aligns correctly with the robot's body for accurate perception.

## 5.3 Depth Camera Simulation

Depth cameras provide 3D distance information for each pixel. Useful for obstacle avoidance and 3D reconstruction:

```bash
<sensor name="depth_camera" type="depth">
  <camera>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
  </camera>
</sensor>
```

- Publishes sensor_msgs/msg/Image or sensor_msgs/msg/PointCloud2 depending on configuration.
- Compatible with ROS 2 perception libraries like PCL for processing point clouds.

## 5.4 IMU Simulation

IMU (Inertial Measurement Unit) provides orientation, angular velocity, and linear acceleration. Used for balance, motion control, and state estimation.

```bash
<sensor name="imu_sensor" type="imu">
  <imu>
    <angular_velocity>
      <x>0</x>
      <y>0</y>
      <z>0</z>
    </angular_velocity>
    <linear_acceleration>
      <x>0</x>
      <y>0</y>
      <z>-9.81</z>
    </linear_acceleration>
  </imu>
</sensor>
```

- Publishes sensor_msgs/msg/Imu on a ROS 2 topic (e.g., /imu/data).
- Critical for humanoid balance, walking, and motion planning.

> Tip: Validate IMU orientation with respect to the robot base frame to avoid incorrect pose estimation.

## 5.5 Accessing and Using Sensor Data

Once sensors are configured, you can read the data in ROS 2 using subscriber nodes:

```bash
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar_scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received {len(msg.ranges)} points")

rclpy.init()
node = LidarReader()
rclpy.spin(node)
```

- Same approach applies for RGB, Depth, and IMU topics.
- Enables integration with perception pipelines, SLAM, and control algorithms.

---

## 5.6 Summary

- Simulated sensors replicate real-world perception in Gazebo.
- LiDAR, RGB, Depth, and IMU sensors provide data on environment, obstacles, and robot motion.
- ROS 2 topics allow easy integration of sensor data into perception and control pipelines.
- Accurate configuration ensures reliable simulation for testing humanoid robotics tasks.

---

## 5.7 Learning Outcomes

After completing this chapter, students will be able to:

- Configure and integrate LiDAR, RGB cameras, Depth cameras, and IMUs in Gazebo simulations.
- Access and interpret simulated sensor data via ROS 2 topics.
- Apply sensor data for perception tasks, including mapping, obstacle detection, and state estimation.
- Validate that simulated sensors reflect realistic behavior for humanoid robots.

---

## References

[1] Koenig, N., Howard, A., “Design and Use Paradigms for Gazebo, an Open-Source Multi-Robot Simulator,” IEEE/RSJ IROS, 2004.

[2] ROS 2 Documentation, “Using Sensors in Gazebo,” https://docs.ros.org/en/humble/Tutorials/Simulation/Gazebo-Sensors.html

[3] PCL Documentation, “Point Cloud Library,” https://pointclouds.org/

[4] Siciliano, B., et al., Springer Handbook of Robotics, 2nd ed., Springer, 2016.

[5] Open Source Robotics Foundation, “Gazebo Sensor Plugins,” http://gazebosim.org/tutorials?tut=plugins_sensors

---

