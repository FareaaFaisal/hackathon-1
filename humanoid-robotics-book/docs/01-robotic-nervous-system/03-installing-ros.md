---
id: 03-installing-ros
title: 'Installing ROS 2'
sidebar_label: 'Installing ROS 2'
---

# Chapter 3: Installing ROS 2

This chapter provides a step-by-step guide to installing ROS 2 on your development machine. We will be using the latest ROS 2 LTS (Long-Term Support) release, which is recommended for most users.

### Prerequisites:

- **Ubuntu 22.04**: ROS 2 is primarily developed and tested on Ubuntu. While it can be installed on other operating systems, we recommend using Ubuntu 22.04 for the best experience.
- **Sudo Privileges**: You will need sudo privileges to install the necessary packages.

### Installation Steps:

1.  **Set Locale**:
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```

2.  **Add the ROS 2 APT Repository**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3.  **Install ROS 2 Packages**:
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    ```

4.  **Source the Setup File**:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    To automatically source the setup file in every new terminal, add this line to your `~/.bashrc` file.

5.  **Install Colcon**:
    Colcon is the build tool for ROS 2. We will use it to build our own ROS 2 packages.
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

After completing these steps, you will have a fully functional ROS 2 installation on your machine. In the next chapter, we will create our first ROS 2 package.
