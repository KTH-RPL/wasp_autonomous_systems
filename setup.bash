#!/bin/bash

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y

# General
sudo apt install build-essential cmake curl git lsb-release openssh-client openssh-server python-is-python3 python3 python3-pip python3-venv pv tig unzip vim wget zip -y

# ROS 2 specific packages
sudo apt install python3-colcon-common-extensions python3-setuptools python3-vcstool ros-jazzy-ros-base -y

# Course dependencies (so we do not need to use Rosdep).
# On native install we would like to have GUI utilities such as RViz and RQT,
# but not here. That is why we do not want to use Rosdep.
sudo apt install ros-jazzy-ros2cli-common-extensions ros-jazzy-foxglove-bridge ros-jazzy-image-transport ros-jazzy-compressed-image-transport ros-jazzy-image-transport-plugins ros-jazzy-teleop-twist-keyboard -y

# assignment_1
sudo apt install ros-jazzy-rclpy ros-jazzy-rosbag2 ros-jazzy-ros2launch ros-jazzy-tf2-ros -y

# assignment_2
sudo apt install python3-opencv ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-rosbag2 ros-jazzy-tf-transformations -y

# assignment_3
sudo apt install ros-jazzy-nav-msgs ros-jazzy-rclpy -y

# assignment_4
# wasp_autonomous_systems_interfaces
sudo apt install ros-jazzy-builtin-interfaces ros-jazzy-controller-manager ros-jazzy-control-msgs ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster ros-jazzy-rclpy ros-jazzy-robot-state-publisher ros-jazzy-ros2launch ros-jazzy-turtlebot3-description -y

# wasp_autonomous_systems_interfaces
sudo apt install ros-jazzy-ament-cmake ros-jazzy-std-msgs ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common -y

# webots_ros2_driver
sudo apt install ros-jazzy-ament-cmake ros-jazzy-python-cmake-module  ros-jazzy-ament-cmake-python ros-jazzy-pluginlib ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-geometry-msgs ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-ros ros-jazzy-vision-msgs ros-jazzy-tinyxml2-vendor libyaml-cpp-dev ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common -y

# webots_ros2_msgs
sudo apt install ros-jazzy-ament-cmake ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime ros-jazzy-builtin-interfaces ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-vision-msgs ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common -y

# webots_ros2_importer
sudo apt install ros-jazzy-builtin-interfaces python3-collada python3-lark ros-jazzy-xacro ros-jazzy-ament-copyright python3-pycodestyle python3-pil python3-numpy python3-pytest -y

# webots_ros2_control
sudo apt install ros-jazzy-hardware-interface ros-jazzy-controller-manager ros-jazzy-pluginlib ros-jazzy-rclcpp ros-jazzy-rclcpp-lifecycle ros-jazzy-ros-environment ros-jazzy-ament-cmake ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common -y

cd /home/ubuntu

curl -L https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/ht25/bashrc >> .bashrc

curl -L -O https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/ht25/rosbags_md5sums.txt

curl -L -O https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/ht25/extract_rosbags.bash

curl -L https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/ht25/setup.status -o setup.log

curl -L -O https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/ht25/wasp_setup.bash

chmod a+x extract_rosbags.bash
chmod a+x wasp_setup.bash

./wasp_setup.bash /home/ubuntu/setup.log
chown root:root /home/ubuntu/setup.log