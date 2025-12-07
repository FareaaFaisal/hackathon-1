---
sidebar_position: 3
title: SDF vs URDF Pipelines
---

## Chapter 3:

Robotics simulation relies on **standardized formats** to describe robot models. Two widely used formats in the ROS 2 ecosystem are:

- **URDF (Unified Robot Description Format)**: Focused on describing the robot’s physical structure, joints, sensors, and kinematics. Primarily used for ROS and visualization tools like RViz.  
- **SDF (Simulation Description Format)**: A more comprehensive format for **simulation environments**, supporting not only robots but also worlds, physics parameters, and lights. Widely used in Gazebo.

This chapter explains the **differences between URDF and SDF**, their use cases, and methods to **convert URDF models into SDF** for Gazebo simulation.

---

## 3.1 Differences Between URDF and SDF

| Feature                  | URDF                                   | SDF                                   |
|---------------------------|----------------------------------------|---------------------------------------|
| Purpose                   | Robot description for ROS             | Full simulation environment description |
| Scope                     | Links, joints, sensors                 | Robots + worlds + lights + physics     |
| Physics Properties        | Limited support                        | Comprehensive support (gravity, friction, etc.) |
| Nested Models             | Not supported                           | Supported                              |
| Extensibility             | Low                                     | High                                    |
| Typical Use               | RViz visualization, ROS controllers    | Gazebo simulation                       |

**Definition:** *URDF*: An XML-based format describing robot kinematics, links, joints, and sensors for ROS.  
**Definition:** *SDF*: A flexible XML-based format describing complete simulation environments, including robots, physics, and lighting.

---

## 3.2 Conversion from URDF to SDF

Many ROS 2 robots are initially defined in URDF. To simulate these robots in Gazebo, we often convert URDF models into SDF.

### 3.2.1 Using `gz sdf` Command

Gazebo provides the `gz sdf` tool for conversion:

```bash
gz sdf -p robot.urdf > robot.sdf
```

- -p flag prints the SDF equivalent of the URDF model.
- Output can then be used in Gazebo world files.

### 3.2.2 Using ROS 2 xacro for Parametric URDF

If the robot is defined with XACRO macros, expand the macros before conversion:

```bash
ros2 run xacro xacro robot.xacro > robot.urdf
gz sdf -p robot.urdf > robot.sdf
```

- XACRO allows parametric modeling, simplifying joint, link, or sensor definitions.
- Conversion preserves joint hierarchies, link geometries, and basic inertial properties.

### 3.2.3 Important Notes

> Tip: SDF supports physics properties and collisions better than URDF, so after conversion, check and adjust:

- Friction coefficients
- Gravity and mass scaling
- Sensor plugins and simulation parameters

> Warning: Some advanced URDF features (custom plugins or non-standard macros) may not convert automatically. Manual adjustments may be needed.

--- 

## 3.3 Integrating SDF Models into Gazebo

Once converted, SDF models can be loaded into Gazebo:

```bash
ros2 launch gazebo_ros empty_world.launch.py world:=my_world.sdf
```

- Robots defined in SDF are fully compatible with Gazebo physics, sensors, and controllers.
- Supports multiple robots, lights, and environmental features in the same world.

---

## 3.4 Summary

1. URDF is focused on robot structure and ROS integration.
2. SDF is designed for full simulation environments with physics and sensors.
3. Conversion from URDF → SDF allows seamless integration of ROS-defined robots into Gazebo simulations.
4. Post-conversion checks ensure proper physics behavior and plugin compatibility.

---

## 3.5 Learning Outcomes

After completing this chapter, students will be able to:

- Identify the differences between URDF and SDF formats.
- Convert URDF robot models to SDF for Gazebo simulation.
- Adjust SDF parameters to ensure proper physics and sensor behavior.
- Integrate converted robots into Gazebo world files.

---

## References

[1] ROS 2 Documentation, “URDF Tutorials,” https://docs.ros.org/en/humble/Tutorials/URDF/Introduction.html

[2] ROS 2 Documentation, “Gazebo Simulation with SDF,” https://docs.ros.org/en/humble/Tutorials/Simulation/Gazebo.html

[3] Koenig, N., Howard, A., “Design and Use Paradigms for Gazebo, an Open-Source Multi-Robot Simulator,” IEEE/RSJ IROS, 2004.
[4] Open Source Robotics Foundation, “SDF Reference,” http://sdformat.org/spec

[5] Siciliano, B., et al., Springer Handbook of Robotics, 2nd ed., Springer, 2016.