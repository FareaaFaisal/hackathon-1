---
sidebar_position: 2
title: Gazebo Setup & Fundamentals
---

## Chapter 2: 

This chapter covers the installation and fundamental usage of Gazebo.

**Gazebo** is a powerful open-source 3D robotics simulator widely used in ROS 2 for testing robot models, sensors, and controllers in realistic environments. It provides physics simulation, sensor data, and visualization tools, enabling safe and efficient development before deploying code to real hardware.

---

## 2.1 Installing Gazebo

Gazebo is compatible with multiple platforms, but Ubuntu 22.04 is recommended for ROS 2 integration.

**Step 1: Setup Gazebo Repository**

```bash
sudo apt update
sudo apt install curl gnupg lsb-release
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
curl -sSL http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
sudo apt update
```


**Step 2: Install Gazebo**

```bash
sudo apt install gazebo11 libgazebo11-dev
```

> Note: Gazebo 11 is the recommended LTS version for ROS 2 Humble. Other versions may require additional compatibility checks.


**Step 3: Install ROS 2 Gazebo Integration Packages**

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```

This allows ROS 2 nodes to communicate with Gazebo for simulation and control.

---

## 2.2 Gazebo Fundamentals
### 2.2.1 World Files

World files define the environment, objects, lighting, and physics properties of a Gazebo simulation. They are written in SDF (Simulation Description Format).

**Example:** Creating a simple world file with a ground plane and a single robot:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://my_robot</uri>
    </include>
  </world>
</sdf>
```

***Definition:*** World File: A configuration file that defines the simulation environment, including terrain, objects, and robots.


### 2.2.2 Gazebo Interface

Gazebo provides multiple panels for simulation control:

1. **World Panel:** Load, pause, or reset the simulation.
2. **Model Panel:** Add, move, or remove objects.
3. **GUI Toolbar:** Camera views, playback controls, and physics parameters.
4. **Console & Logging:** Monitor ROS 2 topics and Gazebo messages.

### 2.2.3 Launching Gazebo

Launch a simple world using ROS 2:

```xml
ros2 launch gazebo_ros empty_world.launch.py
```

This starts Gazebo and integrates ROS 2 nodes for control and simulation.

---

## 2.3 Best Practices

1. **Start simple:** Begin with empty worlds before adding multiple robots or sensors.
2. **Validate physics:** Check that robot motion behaves as expected before running complex scenarios.
3. **Use ROS 2 Topics:** Monitor /joint_states, /odom, and other topics for debugging.
4. **Save frequently:** Export your world files to avoid losing modifications.

---

## 2.4 Summary

Gazebo provides a realistic 3D simulation environment for humanoid robotics development.

World files define the simulation environment and robot placement.

ROS 2 integration allows controlling robots and sensors in simulation.

Understanding the interface and fundamentals is crucial for safe and effective testing before real-world deployment.

---

## 2.5 Learning Outcomes

After completing this chapter, students will be able to:

- Install and configure Gazebo with ROS 2 Humble.
- Load and interact with simulation environments using world files.
- Understand the Gazebo interface and core functionalities.
- Launch simple simulations and verify robot behavior in a safe environment.

---

## References

[1] Koenig, N. & Howard, A., “Design and Use Paradigms for Gazebo, an Open-Source Multi-Robot Simulator,” IEEE/RSJ International Conference on Intelligent Robots and Systems, 2004.

[2] ROS 2 Documentation, “Simulation with Gazebo,” https://docs.ros.org/en/humble/Tutorials/Simulation/Overview.html

[3] Open Source Robotics Foundation, “Gazebo Tutorials,” http://gazebosim.org/tutorials

[4] Siciliano, B., et al., Springer Handbook of Robotics, 2nd ed., Springer, 2016.
