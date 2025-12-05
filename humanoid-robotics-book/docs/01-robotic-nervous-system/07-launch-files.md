---
id: 07-launch-files
title: 'Launch Files'
sidebar_label: 'Launch Files'
---

# Chapter 7: Launch Files

Launch files are a powerful feature of ROS 2 that allow you to start and configure multiple nodes at once. They are written in Python and can be used to automate the process of starting up your robot's software.

### Key Launch File Concepts:

-   **`LaunchDescription`**: The root object of a launch file. It contains a list of actions to be executed.
-   **`Node`**: A launch action that starts a ROS 2 node. You can specify the node's package, executable, name, and parameters.
-   **`IncludeLaunchDescription`**: A launch action that includes another launch file. This allows you to create modular and reusable launch files.
-   **`ExecuteProcess`**: A launch action that executes a shell command.

### Example Launch File:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher',
            name='my_publisher_node',
            parameters=[{'my_param': 'my_value'}]
        ),
        Node(
            package='my_package',
            executable='my_subscriber',
            name='my_subscriber_node'
        )
    ])
```

To run this launch file, you would use the `ros2 launch` command:
```bash
ros2 launch my_package my_launch_file.py
```

In the next chapter, we will explore how to bridge the gap between large language models and ROS 2, enabling natural language interaction with our humanoid robot.
