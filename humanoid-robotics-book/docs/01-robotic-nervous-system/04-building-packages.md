---
id: 04-building-packages
title: 'Building Packages with Colcon'
sidebar_label: 'Building Packages'
---

# Chapter 4: Building Packages with Colcon

In ROS 2, our code is organized into **packages**. A package is a directory containing a collection of related files, such as source code, launch files, configuration files, and a `package.xml` file. The `package.xml` file contains metadata about the package, including its name, version, dependencies, and build instructions.

### Creating a ROS 2 Package:

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

### Building a ROS 2 Package:

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

### Running the Nodes:

Now we can run our publisher and subscriber nodes using the `ros2 run` command.
```bash
ros2 run my_package my_publisher
ros2 run my_package my_subscriber
```

In the next chapter, we will dive deeper into the communication patterns used in ROS 2.
