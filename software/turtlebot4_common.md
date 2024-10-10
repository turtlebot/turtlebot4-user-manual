---
sort: 2
---

# TurtleBot 4 Common

The `turtlebot4` repository contains common packages that are used by both the physical and simulated robot.

Source code is available [here](https://github.com/turtlebot/turtlebot4).

## Installation

```note
The `turtlebot4` packages are automatically installed when either of `turtlebot4_robot` or `turtlebot4_simulator` is installed. Therefore it comes pre-installed on the robot and is installed when going through the [simulation install instructions](./turtlebot4_simulator.md)
```

### Debian package

{% tabs debian %}
{% tab debian galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

Individual packages can be installed through apt:

```bash
sudo apt update
sudo apt install ros-galactic-turtlebot4-description \
ros-galactic-turtlebot4-msgs \
ros-galactic-turtlebot4-navigation \
ros-galactic-turtlebot4-node
```

{% endtab %}
{% tab debian humble %}

Individual packages can be installed through apt:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-description \
ros-humble-turtlebot4-msgs \
ros-humble-turtlebot4-navigation \
ros-humble-turtlebot4-node
```

{% endtab %}
{% tab debian jazzy %}

Individual packages can be installed through apt:

```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot4-description \
ros-jazzy-turtlebot4-msgs \
ros-jazzy-turtlebot4-navigation \
ros-jazzy-turtlebot4-node
```

{% endtab %}
{% endtabs %}

### Source installation

```note
Source installation is an alternative to the debian package and should only be used if the debian package cannot be used or if modifications are being made to the source code.
```

{% tabs source %}
{% tab source galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b galactic
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi --rosdistro galactic
```

Build the packages:

```bash
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
```

Next, the workspace must be sourced on the user computer or on the robot depending on which computer it was installed on.

{% endtab %}
{% tab source humble %}

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b humble
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi --rosdistro humble
```

Build the packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Next, the workspace must be sourced. If implemented on the robot, source the workspace on the robot by running the `turtlebot4-setup` tool and under `ROS Setup` set the workspace as the path to your workspace's setup.bash (`/home/ubuntu/turtlebot4_ws/install/setup.bash`). If being installed on a user computer then this path must be sourced in the terminal or in the user's `.bashrc` file.

{% endtab %}
{% tab source jazzy %}

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b jazzy
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi --rosdistro jazzy
```

Build the packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Next, the workspace must be sourced. If implemented on the robot, source the workspace on the robot by running the `turtlebot4-setup` tool and under `ROS Setup` set the workspace as the path to your workspace's setup.bash (`/home/ubuntu/turtlebot4_ws/install/setup.bash`). If being installed on a user computer then this path must be sourced in the terminal or in the user's `.bashrc` file.

{% endtab %}
{% endtabs %}

## Description

The `turtlebot4_description` package contains the URDF description of the robot and the mesh files for each component.

The description can be published with a `robot_state_publisher` node.

## Messages

The `turtlebot4_msgs` package contains the custom messages used on the TurtleBot 4:
* [UserButton](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_msgs/msg/UserButton.msg): User Button states.
* [UserLed](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_msgs/msg/UserLed.msg): User Led control.
* [UserDisplay](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_msgs/msg/UserDisplay.msg): User Display data.


## Navigation

The `turtlebot4_navigation` packages contains launch and configuration files for using SLAM and navigation on the TurtleBot 4. It also contains the TurtleBot 4 Navigator Python node.

{% tabs navigation %}

{% tab navigation galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

Launch files:

* [Nav Bringup](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/launch/nav_bringup.launch.py): Launches navigation. Allows for launch configurations to use SLAM, Nav2, and localization.
* [SLAM Sync](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/launch/slam_sync.launch.py): Launches `slam_toolbox` with online synchronous mapping. Recommended for use on a PC.
* [SLAM Async](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/launch/slam_async.launch.py): Launches `slam_toolbox` with online asynchronous mapping. Recommended for use on the Raspberry Pi 4.

Running synchronous SLAM:
```bash
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=sync
```

Running asynchronous SLAM with Nav2:
```bash
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=async
```

Running Nav2 with localization and existing map:
```bash
ros2 launch turtlebot4_navigation nav_bringup.launch.py localization:=true slam:=off map:=/path/to/map.yaml
```

{% endtab %}

{% tab navigation humble %}

Launch files:

* [Nav2](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/nav2.launch.py): Launches the Nav2 stack.
* [SLAM](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/slam.launch.py): Launches `slam_toolbox` with online mapping.
* [Localization](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/localization.launch.py): Launches localization on a given map.

Running synchronous SLAM:
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

Running asynchronous SLAM:
```bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=false
```

Running localization with an existing map:
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/path/to/map.yaml
```

Running the Nav2 stack:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

```note
Nav2 requires either SLAM or localization to already be running.
```

{% endtab %}
{% tab navigation jazzy %}

Launch files:

* [Nav2](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/nav2.launch.py): Launches the Nav2 stack.
* [SLAM](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/slam.launch.py): Launches `slam_toolbox` with online mapping.
* [Localization](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/localization.launch.py): Launches localization on a given map.

Running synchronous SLAM:
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

Running asynchronous SLAM:
```bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=false
```

Running localization with an existing map:
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/path/to/map.yaml
```

Running the Nav2 stack:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

```note
Nav2 requires either SLAM or localization to already be running.
```

{% endtab %}
{% endtabs %}

### TurtleBot 4 Navigator

The [TurtleBot 4 Navigator](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/turtlebot4_navigation/turtlebot4_navigator.py) is a Python node that adds TurtleBot 4 specific functionality to the [Nav2 Simple Commander](https://github.com/ros-planning/navigation2/tree/galactic/nav2_simple_commander). It provides a set of Python methods for navigating the TurtleBot 4. This includes docking, navigating to a pose, following waypoints, and more. Visit the [Navigation Tutorials](../tutorials/navigation.html) for examples.

## TurtleBot 4 Node

The `turtlebot4_node` package contains the source code for the [rclcpp](https://github.com/ros2/rclcpp) node `turtlebot4_node` that controls the robots HMI as well as other logic. This node is used by both the physical robot and the simulated robot.

### ROS 2 Interfaces

{% tabs turtlebot4_node %}
{% tab turtlebot4_node galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

**Publishers**

<figure>
    <table>
        <thead>
            <tr>
                <th>Topic</th>
                <th>Message Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>hmi/display</b></td>
                <td><i>turtlebot4_msgs/msg/UserDisplay</i></td>
                <td>The current information that is to be displayed (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>ip</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>The IP address of the Wi-Fi interface</td>
            </tr>
        </tbody>
    </table>
</figure>

**Subscribers**

<figure>
    <table>
        <thead>
            <tr>
                <th>Topic</th>
                <th>Message Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>battery_state</b></td>
                <td><i>sensor_msgs/msg/BatteryState</i></td>
                <td>Current battery state of the Create® 3</td>
            </tr>
            <tr>
                <td><b>hmi/buttons</b></td>
                <td><i>turtlebot4_msgs/msg/UserButton</i></td>
                <td>Button states of the TurtleBot 4 HMI (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>hmi/display/message</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>User topic to print custom message to display (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>hmi/led</b></td>
                <td><i>turtlebot4_msgs/msg/UserLed</i></td>
                <td>User topic to control User LED 1 and 2 (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>interface_buttons</b></td>
                <td><i>irobot_create_msgs/msg/InterfaceButtons</i></td>
                <td>Button states of Create® 3 buttons</td>
            </tr>
            <tr>
                <td><b>joy</b></td>
                <td><i>sensor_msgs/msg/Joy</i></td>
                <td>Bluetooth controller button states (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>wheel_status</b></td>
                <td><i>irobot_create_msgs/msg/WheelStatus</i></td>
                <td>Wheel status reported by Create® 3</td>
            </tr>
        </tbody>
    </table>
</figure>

**Service Clients**

<figure>
    <table>
        <thead>
            <tr>
                <th>Service</th>
                <th>Service Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>e_stop</b></td>
                <td><i>irobot_create_msgs/srv/EStop</i></td>
                <td>Enable or disable motor stop</td>
            </tr>
            <tr>
                <td><b>robot_power</b></td>
                <td><i>irobot_create_msgs/srv/RobotPower</i></td>
                <td>Power off the robot</td>
            </tr>
            <tr>
                <td><b>start_motor</b></td>
                <td><i>std_srvs/srv/Empty</i></td>
                <td>Start the RPLIDAR motor</td>
            </tr>
            <tr>
                <td><b>stop_motor</b></td>
                <td><i>std_srvs/srv/Empty</i></td>
                <td>Stop the RPLIDAR motor</td>
            </tr>
        </tbody>
    </table>
</figure>

**Action Clients**

<figure>
    <table>
        <thead>
            <tr>
                <th>Action</th>
                <th>Action Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>dock</b></td>
                <td><i>irobot_create_msgs/action/DockServo</i></td>
                <td>Command the robot to dock into its charging station</td>
            </tr>
            <tr>
                <td><b>wall_follow</b></td>
                <td><i>irobot_create_msgs/action/WallFollow</i></td>
                <td>Command the robot to wall follow on left or right side using bump and IR sensors</td>
            </tr>
            <tr>
                <td><b>undock</b></td>
                <td><i>irobot_create_msgs/action/Undock</i></td>
                <td>Command the robot to undock from its charging station</td>
            </tr>
        </tbody>
    </table>
</figure>

{% endtab %}
{% tab turtlebot4_node humble %}

**Publishers**

<figure>
    <table>
        <thead>
            <tr>
                <th>Topic</th>
                <th>Message Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>hmi/display</b></td>
                <td><i>turtlebot4_msgs/msg/UserDisplay</i></td>
                <td>The current information that is to be displayed (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>ip</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>The IP address of the Wi-Fi interface</td>
            </tr>
            <tr>
                <td><b>function_calls</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>Publishes the name of a button or menu function when it is called</td>
            </tr>
        </tbody>
    </table>
</figure>

**Subscribers**

<figure>
    <table>
        <thead>
            <tr>
                <th>Topic</th>
                <th>Message Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>battery_state</b></td>
                <td><i>sensor_msgs/msg/BatteryState</i></td>
                <td>Current battery state of the Create® 3</td>
            </tr>
            <tr>
                <td><b>hmi/buttons</b></td>
                <td><i>turtlebot4_msgs/msg/UserButton</i></td>
                <td>Button states of the TurtleBot 4 HMI (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>hmi/display/message</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>User topic to print custom message to display (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>hmi/led</b></td>
                <td><i>turtlebot4_msgs/msg/UserLed</i></td>
                <td>User topic to control User LED 1 and 2 (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>interface_buttons</b></td>
                <td><i>irobot_create_msgs/msg/InterfaceButtons</i></td>
                <td>Button states of Create® 3 buttons</td>
            </tr>
            <tr>
                <td><b>joy</b></td>
                <td><i>sensor_msgs/msg/Joy</i></td>
                <td>Bluetooth controller button states</td>
            </tr>
            <tr>
                <td><b>wheel_status</b></td>
                <td><i>irobot_create_msgs/msg/WheelStatus</i></td>
                <td>Wheel status reported by Create® 3</td>
            </tr>
        </tbody>
    </table>
</figure>

**Service Clients**

<figure>
    <table>
        <thead>
            <tr>
                <th>Service</th>
                <th>Service Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>e_stop</b></td>
                <td><i>irobot_create_msgs/srv/EStop</i></td>
                <td>Enable or disable motor stop</td>
            </tr>
            <tr>
                <td><b>robot_power</b></td>
                <td><i>irobot_create_msgs/srv/RobotPower</i></td>
                <td>Power off the robot</td>
            </tr>
            <tr>
                <td><b>start_motor</b></td>
                <td><i>std_srvs/srv/Empty</i></td>
                <td>Start the RPLIDAR motor</td>
            </tr>
            <tr>
                <td><b>stop_motor</b></td>
                <td><i>std_srvs/srv/Empty</i></td>
                <td>Stop the RPLIDAR motor</td>
            </tr>
        </tbody>
    </table>
</figure>

**Action Clients**

<figure>
    <table>
        <thead>
            <tr>
                <th>Action</th>
                <th>Action Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>dock</b></td>
                <td><i>irobot_create_msgs/action/Dock</i></td>
                <td>Command the robot to dock into its charging station</td>
            </tr>
            <tr>
                <td><b>wall_follow</b></td>
                <td><i>irobot_create_msgs/action/WallFollow</i></td>
                <td>Command the robot to wall follow on left or right side using bump and IR sensors</td>
            </tr>
            <tr>
                <td><b>undock</b></td>
                <td><i>irobot_create_msgs/action/Undock</i></td>
                <td>Command the robot to undock from its charging station</td>
            </tr>
        </tbody>
    </table>
</figure>

{% endtab %}
{% tab turtlebot4_node jazzy %}

**Publishers**

<figure>
    <table>
        <thead>
            <tr>
                <th>Topic</th>
                <th>Message Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>hmi/display</b></td>
                <td><i>turtlebot4_msgs/msg/UserDisplay</i></td>
                <td>The current information that is to be displayed (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>ip</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>The IP address of the Wi-Fi interface</td>
            </tr>
            <tr>
                <td><b>function_calls</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>Publishes the name of a button or menu function when it is called</td>
            </tr>
        </tbody>
    </table>
</figure>

**Subscribers**

<figure>
    <table>
        <thead>
            <tr>
                <th>Topic</th>
                <th>Message Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>battery_state</b></td>
                <td><i>sensor_msgs/msg/BatteryState</i></td>
                <td>Current battery state of the Create® 3</td>
            </tr>
            <tr>
                <td><b>hmi/buttons</b></td>
                <td><i>turtlebot4_msgs/msg/UserButton</i></td>
                <td>Button states of the TurtleBot 4 HMI (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>hmi/display/message</b></td>
                <td><i>std_msgs/msg/String</i></td>
                <td>User topic to print custom message to display (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>hmi/led</b></td>
                <td><i>turtlebot4_msgs/msg/UserLed</i></td>
                <td>User topic to control User LED 1 and 2 (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>interface_buttons</b></td>
                <td><i>irobot_create_msgs/msg/InterfaceButtons</i></td>
                <td>Button states of Create® 3 buttons</td>
            </tr>
            <tr>
                <td><b>joy</b></td>
                <td><i>sensor_msgs/msg/Joy</i></td>
                <td>Bluetooth controller button states</td>
            </tr>
            <tr>
                <td><b>wheel_status</b></td>
                <td><i>irobot_create_msgs/msg/WheelStatus</i></td>
                <td>Wheel status reported by Create® 3</td>
            </tr>
        </tbody>
    </table>
</figure>

**Service Clients**

<figure>
    <table>
        <thead>
            <tr>
                <th>Service</th>
                <th>Service Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>e_stop</b></td>
                <td><i>irobot_create_msgs/srv/EStop</i></td>
                <td>Enable or disable motor stop</td>
            </tr>
            <tr>
                <td><b>robot_power</b></td>
                <td><i>irobot_create_msgs/srv/RobotPower</i></td>
                <td>Power off the robot</td>
            </tr>
            <tr>
                <td><b>start_motor</b></td>
                <td><i>std_srvs/srv/Empty</i></td>
                <td>Start the RPLIDAR motor</td>
            </tr>
            <tr>
                <td><b>stop_motor</b></td>
                <td><i>std_srvs/srv/Empty</i></td>
                <td>Stop the RPLIDAR motor</td>
            </tr>
        </tbody>
    </table>
</figure>

**Action Clients**

<figure>
    <table>
        <thead>
            <tr>
                <th>Action</th>
                <th>Action Type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>dock</b></td>
                <td><i>irobot_create_msgs/action/Dock</i></td>
                <td>Command the robot to dock into its charging station</td>
            </tr>
            <tr>
                <td><b>wall_follow</b></td>
                <td><i>irobot_create_msgs/action/WallFollow</i></td>
                <td>Command the robot to wall follow on left or right side using bump and IR sensors</td>
            </tr>
            <tr>
                <td><b>undock</b></td>
                <td><i>irobot_create_msgs/action/Undock</i></td>
                <td>Command the robot to undock from its charging station</td>
            </tr>
        </tbody>
    </table>
</figure>

{% endtab %}
{% endtabs %}

### Functions

The node has a set of static functions that can be used either with a button or through the display menu.

<figure>
    <table>
        <thead>
            <tr>
                <th>Function name</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>Dock</b></td>
                <td>Call the <i>dock</i> action</td>
            </tr>
            <tr>
                <td><b>Undock</b></td>
                <td>Call the <i>undock</i> action</td>
            </tr>
            <tr>
                <td><b>Wall Follow Left</b></td>
                <td>Call the <i>wall_follow</i> action with direction <b>FOLLOW_LEFT</b> and a duration of 10 seconds.</td>
            </tr>
            <tr>
                <td><b>Wall Follow Right</b></td>
                <td>Call the <i>wall_follow</i> action with direction <b>FOLLOW_RIGHT</b> and a duration of 10 seconds.</td>
            </tr>
            <tr>
                <td><b>Power</b></td>
                <td>Call the <i>robot_power</i> service to power off the robot</td>
            </tr>
            <tr>
                <td><b>EStop</b></td>
                <td>Call the <i>e_stop</i> action to toggle the EStop state</td>
            </tr>
            <tr>
                <td><b>Scroll Up</b></td>
                <td>Scroll menu up (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>Scroll Down</b></td>
                <td>Scroll menu down (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>Back</b></td>
                <td>Exit message screen or return to first menu entry (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>Select</b></td>
                <td>Select currently highlighted menu entry (TurtleBot 4 model only)</td>
            </tr>
            <tr>
                <td><b>Help</b></td>
                <td>Print help statement on the display (TurtleBot 4 model only)</td>
            </tr>
        </tbody>
    </table>
</figure>

### Configuration

This node can be configured using a parameter *.yaml* file. The default robot parameters can be found [here](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/config/turtlebot4.yaml).

**Parameters**

<figure>
    <table>
        <thead>
            <tr>
                <th>Parameter</th>
                <th>Parameter type</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>wifi.interface</b></td>
                <td><i>String</i></td>
                <td>The Wi-Fi interface being used by the computer. This is used to find the current IP address of the computer</td>
            </tr>
            <tr>
                <td><b>menu.entries</b></td>
                <td><i>List of Strings</i></td>
                <td>Set menu entries to be displayed</td>
            </tr>
            <tr>
                <td><b>buttons</b></td>
                <td><i>Key Value pairs</i></td>
                <td>Set the function of Create® 3 and HMI buttons</td>
            </tr>
            <tr>
                <td><b>controller</b></td>
                <td><i>Key Value pairs</i></td>
                <td>Set the function of TurtleBot 4 Controller buttons</td>
            </tr>
        </tbody>
    </table>
</figure>


### Buttons

The `Buttons` class in `turtlebot4_node` provides functionality to all buttons on the robot. This includes the Create® 3 buttons, HMI buttons, and TurtleBot 4 Controller buttons.
The node receives button states from the *interface_buttons*, *hmi/buttons*, and *joy* topics.

Each button can be configured to have either a single function when pressed, or two functions by using a short or long press. This is done through [configuration](#configuration).

Supported buttons:

```yaml
buttons:
    create3_1:
    create3_power:
    create3_2:
    hmi_1:
    hmi_2:
    hmi_3:
    hmi_4:

controller:
    a:
    b:
    x:
    y:
    up:
    down:
    left:
    right:
    l1:
    l2:
    l3:
    r1:
    r2:
    r3:
    share:
    options:
    home:
```

#### Example

Lets say we want the TurtleBot 4 to have the following button functions:
- Make a short press of Create® 3 button 1 toggle EStop.
- Power off robot with 5 second press of Home on the TurtleBot 4 Controller.
- Short press of HMI button 1 performs Wall Follow Left, long press of 3 seconds performs Wall Follow Right.

Create a new yaml file:

```bash
cd /home/ubuntu/turtlebot4_ws
touch example.yaml
```

Use your favourite text editor and paste the following into `example.yaml`:
```yaml
turtlebot4_node:
  ros__parameters:
    buttons:
      create3_1: ["EStop"]
      hmi_1: ["Wall Follow Left", "Wall Follow Right", "3000"]

    controller:
      home: ["Power", "5000"]
```

Launch the robot with your new configuration:

```bash
ros2 launch turtlebot4_bringup standard.launch.py param_file:=/home/ubuntu/turtlebot4_ws/example.yaml
```

The buttons should now behave as described in `example.yaml`.

### LEDs

The `Leds` class in `turtlebot4_node` controls the states of the HMI LEDs on the TurtleBot 4. It is not used for the TurtleBot 4 Lite.

#### Status LEDs

The status LEDs are controlled by the `turtlebot4_node`.

<figure>
    <table>
        <thead>
            <tr>
                <th>LED</th>
                <th>Colour</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>POWER</b></td>
                <td><i>Green</i></td>
                <td>Always ON</td>
            </tr>
            <tr>
                <td><b>MOTOR</b></td>
                <td><i>Green</i></td>
                <td>ON when wheels are enabled, OFF when wheels are disabled</td>
            </tr>
            <tr>
                <td><b>COMMS</b></td>
                <td><i>Green</i></td>
                <td>ON when communication with the Create® 3 is active. OFF otherwise</td>
            </tr>
            <tr>
                <td><b>WIFI</b></td>
                <td><i>Green</i></td>
                <td>ON when a valid IP address can be found for the specified Wi-Fi interface</td>
            </tr>
            <tr>
                <td><b>BATTERY</b></td>
                <td><i>Green, Yellow, Red</i></td>
                <td>Colour will reflect battery percentage</td>
            </tr>
        </tbody>
    </table>
</figure>

**Battery LED state**
<figure>
    <table>
        <thead>
            <tr>
                <th>Battery Percentage</th>
                <th>Colour</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <th>50-100</th>
                <th>Green</th>
            </tr>
            <tr>
                <th>20-50</th>
                <th>Yellow</th>
            </tr>
            <tr>
                <th>12-20</th>
                <th>Red</th>
            </tr>
            <tr>
                <th>0-12</th>
                <th>Blinking Red</th>
            </tr>
        </tbody>
    </table>
</figure>

#### User LEDs

The user LEDs can be set by publishing to the **hmi/led** topic with a [UserLed](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_msgs/msg/UserLed.msg) message.

<figure>
    <table>
        <thead>
            <tr>
                <th>LED</th>
                <th>Colour</th>
                <th>Description</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>USER_1</b></td>
                <td><i>Green</i></td>
                <td>User Controlled</td>
            </tr>
            <tr>
                <td><b>USER_2</b></td>
                <td><i>Green, Yellow, Red</i></td>
                <td>User Controlled</td>
            </tr>
        </tbody>
    </table>
</figure>

#### Examples

Set `USER_1` to solid green:

```bash
ros2 topic pub /hmi/led turtlebot4_msgs/msg/UserLed "led: 0
color: 1
blink_period: 1000
duty_cycle: 1.0" --once
```

<figure class="aligncenter">
    <img src="media/user_1_green.png" alt="User 1 Green" style="width: 35%"/>
    <figcaption>User 1: Solid Green</figcaption>
</figure>

Set `USER_1` OFF:

```bash
ros2 topic pub /hmi/led turtlebot4_msgs/msg/UserLed "led: 0
color: 0
blink_period: 1000
duty_cycle: 1.0" --once
```

<figure class="aligncenter">
    <img src="media/user_1_off.png" alt="User 1 Off" style="width: 35%"/>
    <figcaption>User 1: Off</figcaption>
</figure>

Blink `USER_2` red at 1hz with 50% duty cycle:

```bash
ros2 topic pub /hmi/led turtlebot4_msgs/msg/UserLed "led: 1
color: 2
blink_period: 1000
duty_cycle: 0.5" --once
```

<figure class="aligncenter">
    <img src="media/user_2_red_blink.gif" alt="User 2 Red Blink" style="width: 35%"/>
    <figcaption>User 2: Red, 1hz, 50%</figcaption>
</figure>

### Display

The `Display` class in `turtlebot4_node` controls the HMI display of the TurtleBot 4. The physical display is a 128x64 OLED which is controlled over I2C with a SSD1306 driver.

The display has a header line which contains the IP address of the Wi-Fi interface specified in [configuration](#configuration), as well as the battery percentage received on the */battery_state* topic. The display also has 5 additional lines which are used for the menu by default. The menu entries are specified in [configuration](#configuration) and are a set of the available functions. The 5 menu lines can be overwritten by publishing to the */hmi/display/message* with a **String** message.

```note
The menu can have any number of entries. If there are more than 5 entries, the user will have to scroll down to see the entries that do not fit on the 5 menu lines.
```

#### Menu Control

The TurtleBot 4 display has a simple scrolling menu. There are 4 control functions for the menu: Scroll up, Scroll down, Select, and Back.

* Scroll up and down allow the users to navigate through the menu entries and by default are mapped to user buttons 3 and 4 respectively.
* The select function will call the currently selected menu entry. This can trigger an action such as docking, a service such as EStop, or display a message such as the Help message. This function is mapped to user button 1 by default.
* The back function allows the user to return back to the menu from a message screen. If the menu is already showing the menu entries, it will return to showing the first 5 menu entries and the first entry will be highlighted.

<figure class="aligncenter">
    <img src="media/menu_control.gif" alt="TurtleBot 4 Menu Controls" style="width: 35%"/>
    <figcaption>TurtleBot 4 Menu Controls</figcaption>
</figure>