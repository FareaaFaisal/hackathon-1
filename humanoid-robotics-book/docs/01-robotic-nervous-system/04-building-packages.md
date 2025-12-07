---
id: 04-building-packages
title: 'Building Packages with Colcon'
sidebar_label: 'Building Packages'
---

# Chapter 4: Building Packages with Colcon

## 4.1 Overview

In ROS 2, **code is organized into packages**. A package is a directory containing a collection of related files such as source code, launch files, configuration files, and the `package.xml` file. The `package.xml` contains metadata about the package, including its name, version, dependencies, and build instructions.

Using **Colcon**, the ROS 2 build tool, we can compile and manage these packages efficiently in a workspace. This chapter guides you through creating, building, and running your first ROS 2 packages.

---

## 4.2 ROS 2 Package Structure

A typical ROS 2 package includes:

- **Source Code**: Python or C++ nodes.
- **Launch Files**: For starting nodes and configuring parameters.
- **Configuration Files**: YAML or other configuration formats.
- **package.xml**: Metadata and dependencies for the build system.

**Definition:** *ROS 2 Package*: A structured directory containing nodes, configuration, and metadata, enabling modular development and reuse.


## 4.3 Creating a ROS 2 Package:

1.  **Create a Workspace**:
    A workspace is a directory where you can create and manage your ROS 2 packages.
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2.  **Create a Package**:
    We will use the `ros2 pkg create` command to create a new package. Let's create a simple publisher/subscriber package.
    ```bash
    cd src
    ros2 pkg create --build-type ament_cmake --node-name my_publisher my_package
    ros2 pkg create --build-type ament_cmake --node-name my_subscriber my_package
    ```
    This will create a new directory called `my_package` with the necessary files and subdirectories.

## 4.4 Building a ROS 2 Package:

1.  **Build with Colcon**:
    `colcon` is the build tool for ROS 2. To build our package, we will run `colcon build` from the root of our workspace.
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
    This will build all the packages in the `src` directory and generate the necessary files in the `build`, `install`, and `log` directories.

2.  **Source the Workspace**:
    After building the package, we need to source the workspace's setup file to make the executables and other resources available in our environment.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## 4.5 Running the Nodes:

Now we can run our publisher and subscriber nodes using the `ros2 run` command.
```bash
ros2 run my_package my_publisher
ros2 run my_package my_subscriber
```
- Publisher node sends messages to a topic.
- Subscriber node receives messages from the same topic.
- Demonstrates basic ROS 2 communication patterns.

## 4.6 Summary

- ROS 2 packages modularize code and resources.
- Workspaces allow multiple packages to be built and managed together.
- ros2 pkg create generates standard package structures.
- Colcon builds packages, handling dependencies and creating installable artifacts.
- Nodes can be executed with ros2 run after sourcing the workspace.

## 4.7 Learning Outcomes

After completing this chapter, students will be able to:

- Create a ROS 2 workspace and understand its directory structure.
- Generate new ROS 2 packages with ros2 pkg create.
- Build packages using Colcon and source the workspace.
- Run publisher and subscriber nodes to test communication in ROS 2.

## References

[1] M. Quigley et al., “ROS: An Open-Source Robot Operating System,” ICRA Workshop on Open Source Software, 2009.
[2] ROS 2 Documentation, “Creating a ROS 2 Package,” https://docs.ros.org/en/humble/Creating-A-Workspace.html
.
[3] Fox, D., et al. Probabilistic Robotics. MIT Press, 2009.


