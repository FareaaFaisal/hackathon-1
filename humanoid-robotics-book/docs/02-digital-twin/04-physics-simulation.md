---
sidebar_position: 4
title: Physics Simulation (Rigid Bodies, Collisions, Gravity)
---

## Chapter 04:

This chapter focuses on configuring physics engines and validating humanoid balance simulations.

Accurate physics simulation is essential for testing **humanoid robots** in a virtual environment. By simulating rigid body dynamics, collisions, and gravity, engineers can validate balance, motion planning, and interaction with the environment before deploying to real hardware.

---

## 4.1 Physics Engines in Gazebo

Gazebo supports multiple physics engines, including:

- **ODE (Open Dynamics Engine)** – default, general-purpose rigid body simulation.  
- **Bullet** – supports advanced collision and contact modeling.  
- **DART (Dynamic Animation and Robotics Toolkit)** – ideal for humanoid and legged robots.  
- **Simbody** – accurate multibody simulation with advanced constraints.

### Selecting a Physics Engine

Physics engines are configured in **world files** using the `<physics>` tag:

```xml
<physics type="ode" name="default_physics">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <gravity>0 0 -9.81</gravity>
</physics>
```

- max_step_size: The simulation time step for integration. Smaller values increase accuracy but reduce speed.
- real_time_factor: Ratio of simulation time to real time.
- gravity: Sets the gravity vector (m/s²).

***Definition:*** Physics Engine: Software component that simulates forces, collisions, and rigid body dynamics.

---

## 4.2 Rigid Body Dynamics

A rigid body is an idealized physical object that does not deform under forces. Each link of a humanoid robot is modeled as a rigid body with mass, inertia, and collision properties.

```xml
<link name="torso_link">
  <inertial>
    <mass value="10"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

- **Mass:** Influences acceleration and stability.
- **Inertia matrix:** Determines resistance to rotational motion.
- Accurate values are essential to maintain humanoid balance.

---

## 4.3 Collisions

Collisions define how rigid bodies interact when they come into contact. Gazebo uses collision geometry defined in links:

```xml
<collision>
  <geometry>
    <box size="0.5 0.2 0.8"/>
  </geometry>
</collision>
```

- Correct collision shapes prevent interpenetration of links.
- Simplified collision geometry can improve simulation speed but may reduce realism.
- Use visual meshes for rendering and collision meshes for physics calculations.

***Definition:*** Collision Shape: The geometric approximation of a rigid body used by the physics engine to detect and respond to contact events.

---

## 4.4 Gravity

Gravity ensures realistic motion and humanoid balance. Standard gravity is -9.81 m/s² along the Z-axis. Proper gravity configuration allows:

- Testing standing, walking, and dynamic movements.
- Evaluating stability under perturbations.
- Planning realistic trajectories and control strategies.

> Tip: Always verify that the humanoid's center of mass is correctly modeled to prevent unrealistic tipping or sliding.

---

## 4.5 Validating Humanoid Balance

1. **Static Balance Test:** Ensure the robot maintains upright posture when standing.
2. **Dynamic Balance Test:** Simulate walking or shifting weight using joint controllers.
3. **Collision Response:** Push the robot virtually and observe correct physics-based reactions.
4. **Inertia Check:** Verify that heavy links react realistically to applied forces.

**Example:** In Gazebo, apply a force to the torso:

```xml
ros2 topic pub /torso_link/force geometry_msgs/msg/Wrench "{force: {x: 50, y: 0, z: 0}, torque: {x:0, y:0, z:0}}"
```

Observe how the robot’s joints and base respond according to mass and inertia.

---

## 4.6 Summary

- Physics engines simulate rigid bodies, collisions, and gravity to enable realistic humanoid behavior.
- Accurate mass, inertia, and collision properties are critical for humanoid balance.
- Validating static and dynamic balance ensures that robots perform safely in real-world environments.
- Proper configuration of step size, gravity, and real-time factors improves simulation fidelity.

---

## 4.7 Learning Outcomes

After completing this chapter, students will be able to:

1. Configure and select suitable physics engines for humanoid robotics.
2. Understand rigid body dynamics and their importance in robot balance.
3. Set up collision shapes and validate realistic interaction with the environment.
4. Adjust gravity and simulation parameters for accurate humanoid motion.
5. Validate the physical accuracy of humanoid robot models in simulation.

---

## References

[1] Koenig, N., Howard, A., “Design and Use Paradigms for Gazebo, an Open-Source Multi-Robot Simulator,” IEEE/RSJ IROS, 2004.

[2] Siciliano, B., et al., Springer Handbook of Robotics, 2nd ed., Springer, 2016.

[3] ROS 2 Documentation, “Gazebo Physics Properties,” https://docs.ros.org/en/humble/Tutorials/Simulation/Gazebo-Physics.html

[4] Open Dynamics Engine Documentation, https://www.ode.org/ode-latest-userguide.html

[5] DART Physics Engine Documentation, https://dartsim.github.io/website/

---
