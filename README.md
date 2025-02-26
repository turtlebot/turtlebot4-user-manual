# Turtlebot4 User Manual

This repository contains the source code for building the Turtlebot4 User Manual. To view the manual, click [here](https://turtlebot.github.io/turtlebot4-user-manual/).

## Build

To build the Turtlebot4's User Manual locally, first [install Ruby version 3.2](https://gorails.com/setup/ubuntu/24.04)

You can check your ruby the version using:
```bash
ruby -v
```
Ensure that it is showing the correct version.

<p>
<details>
    <summary><b>Configuring Ubuntu 24.04</b></summary>
    By default, Ubuntu 24.04 uses a debian-packaged version of Ruby, which installs gems to a root-owned path. This makes installing additional gems problematic.
    To work around this issue, run the following commands:
    ```shell
    mkdir $HOME/.ruby
    echo 'export GEM_HOME=$HOME/.ruby/' >> $HOME/.bashrc
    echo 'export PATH="$PATH:$HOME/.ruby/bin"' >> $HOME/.bashrc
    source $HOME/.bashrc
    ```
    This will make the `gem` command install Ruby gems to your local user's `.ruby` directory.
</details>
</p>

Ensure your Ruby gems are up to date:
```shell
gem update --system
```

Clone this repository:

```bash
git clone https://github.com/turtlebot/turtlebot4-user-manual.git
```

Build and start a local server:

```bash
cd turtlebot4-user-manual
make
make update
make server
```

The webpage will now be available at `http://127.0.0.1:4000/turtlebot4-user-manual`


 # ROS 2 Humble Install

 Set Locale
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