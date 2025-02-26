---
sort: 3
---

# ROS 2 Setup

# Set Locale
```bash
locale
```
# Add ROS2 apt repository


```bash
sudo apt install software-properties-common
```

```bash
sudo add-apt-repository universe
```

## ROS 2 GPG Key

```bash
sudo apt update && sudo apt install curl -y
```

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

## Add repo to sources list

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

# Install ROS 2 packages

```bash
sudo apt update
```

## ROS 2 packages

```bash
sudo apt upgrade
```

## Desktop Install

```bash
sudo apt install ros-humble-desktop
```

## ROS Base

```bash
sudo apt install ros-humble-ros-base
```

## Developer Tools

```bash
sudo apt install ros-dev-tools
```

# Environment Setup

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Source .bashrc
```bash
source ~/.bashrc
```
