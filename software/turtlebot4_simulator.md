---
sort: 4
---

# TurtleBot 4 Simulator

The `turtlebot4_simulator` metapackage contains packages used to simulate the TurtleBot 4 in Ignition Gazebo.

Source code is available [here](https://github.com/turtlebot/turtlebot4_simulator).

## Installation

```note
The `turtlebot4_simulator` metapackage can be installed on a PC running Ubuntu Desktop 20.04 with ROS 2 Galactic or Ubuntu Desktop 22.04 with ROS 2 Humble.
```

### Dev Tools

Install useful development tools:

```bash
sudo apt install ros-dev-tools
```

### Gazebo

{% tabs ignition %}
{% tab ignition galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

Ignition Edifice must be installed:

```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-edifice
```

{% endtab %}
{% tab ignition humble %}

Ignition Fortress must be installed:

```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

{% endtab %}
{% tab ignition jazzy %}

Gazebo Harmonic [must be installed](https://gazebosim.org/docs/latest/install_ubuntu/):

```bash
sudo apt-get install curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

{% endtab %}
{% endtabs %}

### Debian package

{% tabs debian %}
{% tab debian galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

To install the metapackage through apt:

```bash
sudo apt update
sudo apt install ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes
```

{% endtab %}
{% tab debian humble %}

To install the metapackage through apt:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator
```

{% endtab %}
{% tab debian jazzy %}

To install the metapackage through apt:

```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot4-simulator
```

{% endtab %}
{% endtabs %}

### Source installation

```note
Source installation is an alternative to the debian package and should only be used if the debian package cannot be used or if modifications are being made to the source code.
```

{% tabs setup %}
{% tab setup galactic %}

> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b galactic
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
vcs import src < src/turtlebot4_simulator/dependencies.repos
rosdep install --from-path src -yi
```

Build the packages:

```bash
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
```

Next, the workspace must be sourced by running `source ~/turtlebot4_ws/install/setup.bash` in the terminal or by adding that command in the `.bashrc` file and sourcing the `.bashrc` file.

{% endtab %}
{% tab setup humble %}

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```

Build the packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Next, the workspace must be sourced by running `source ~/turtlebot4_ws/install/setup.bash` in the terminal or by adding that command in the `.bashrc` file and sourcing the `.bashrc` file.

{% endtab %}
{% tab setup jazzy %}

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```

Build the packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Next, the workspace must be sourced by running `source ~/turtlebot4_ws/install/setup.bash` in the terminal or by adding that command in the `.bashrc` file and sourcing the `.bashrc` file.

{% endtab %}
{% endtabs %}

## Networking

Any ROS 2 networking settings on the computer apply to all nodes launched including the simulation nodes. It is recommended to configure the computer for simple discovery prior to running any simulations. If the computer is on the same network as other ROS nodes unrelated to the simulation, then the `LOCAL_HOST_ONLY` environment variable should be set to `True`.

The simulation can be run in discovery server mode but the discovery server referenced must exist and be accessible for discovery as well as not having any conflicts in the topics launched.

``` warning
Simulated robots and physical robots should not be combined in the same system.
```

## Gazebo Bringup

{% tabs bringup %}
{% tab bringup galactic %}
> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

The `turtlebot4_ignition_bringup` package contains launch files and configurations to launch Ignition Gazebo.

Launch files:

* [Ignition](https://github.com/turtlebot/turtlebot4_simulator/blob/galactic/turtlebot4_ignition_bringup/launch/ignition.launch.py): Launches Ignition Gazebo and all required nodes to run the simulation.
* [ROS Ignition Bridge](https://github.com/turtlebot/turtlebot4_simulator/blob/galactic/turtlebot4_ignition_bringup/launch/ros_ign_bridge.launch.py): Launches all of the required `ros_ign_bridge` nodes to bridge Ignition topics with ROS topics.
* [TurtleBot 4 Nodes](https://github.com/turtlebot/turtlebot4_simulator/blob/galactic/turtlebot4_ignition_bringup/launch/turtlebot4_nodes.launch.py): Launches the `turtlebot4_node` and `turtlebot4_ignition_hmi_node` required to control the HMI plugin and robot behaviour.

Ignition launch configuration options:
- **model**: Which TurtleBot 4 model to use.
    - options: *standard, lite*
    - default: *standard*
- **rviz**: Whether to launch rviz.
    - options: *true, false*
    - default: *false*
- **slam**: Whether to launch SLAM.
    - options: *off, sync, async*
    - default: *off*
- **nav2**: Whether to launch Nav2.
    - options: *true, false*
    - default: *false*
- **param_file**: Path to parameter file for `turtlebot4_node`.
    - default: */path/to/turtlebot4_ignition_bringup/config/turtlebot4_node.yaml*
- **world**: Which world to use for simulation.
    - default: *depot*
- **robot_name**: What to name the spawned robot.
    - default: *turtlebot4*

Running the simulator with default settings:
```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py
```

Running synchronous SLAM with Nav2:
```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py slam:=sync nav2:=true rviz:=true
```

{% endtab %}
{% tab bringup humble %}
The `turtlebot4_ignition_bringup` package contains launch files and configurations to launch Ignition Gazebo.

Launch files:

* [Turtlebot 4 Ignition Launch](https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py): Launches Ignition Gazebo and all required nodes to run the simulation.
* [Ignition](https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/ignition.launch.py): Launches Ignition Gazebo only.
* [ROS Ignition Bridge](https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/ros_ign_bridge.launch.py): Launches all of the required `ros_ign_bridge` nodes to bridge Ignition topics with ROS topics.
* [TurtleBot 4 Nodes](https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_nodes.launch.py): Launches the `turtlebot4_node` and `turtlebot4_ignition_hmi_node` required to control the HMI plugin and robot behaviour.

Turtlebot 4 Ignition launch configuration options:

- **model**: Which TurtleBot 4 model to use
    - options: *standard, lite*
    - default: *standard*
- **rviz**: Whether to launch rviz
    - options: *true, false*
    - default: *false*
- **localization**: Whether to launch localization
    - options: *true, false*
    - default: *false*
- **slam**: Whether to launch SLAM
    - options: *true, false*
    - default: *false*
- **nav2**: Whether to launch Nav2
    - options: *true, false*
    - default: *false*
- **world**: Which world to use for simulation
    - options: *depot, maze, warehouse*
    - default: *warehouse*
- **namespace**: Optional robot namespace
  - options: Any valid ROS 2 name as a string
  - default: blank ("")
- **x**: x coordinate of the robot and dock spawn location in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*
- **y**: y coordinate of the robot and dock spawn location in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*
- **z**: z coordinate of the robot and dock spawn location in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*
- **yaw**: robot and dock orientation at spawn in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*

Running the simulator with default settings:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

Running synchronous SLAM with Nav2:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

{% endtab %}
{% tab bringup jazzy %}
The `turtlebot4_gz_bringup` package contains launch files and configurations to launch Gazebo.

Launch files:

* [Turtlebot 4 Gazebo Launch](https://github.com/turtlebot/turtlebot4_simulator/blob/jazzy/turtlebot4_gz_bringup/launch/turtlebot4_gz.launch.py): Launches Gazebo and all required nodes to run the simulation.
* [Gazebo](https://github.com/turtlebot/turtlebot4_simulator/blob/jazzy/turtlebot4_gz_bringup/launch/sim.launch.py): Launches Gazebo only.
* [ROS Gazebo Bridge](https://github.com/turtlebot/turtlebot4_simulator/blob/jazzy/turtlebot4_gz_bringup/launch/ros_gz_bridge.launch.py): Launches all of the required `ros_gz_bridge` nodes to bridge Ignition topics with ROS topics.
* [TurtleBot 4 Nodes](https://github.com/turtlebot/turtlebot4_simulator/blob/jazzy/turtlebot4_gz_bringup/launch/turtlebot4_nodes.launch.py): Launches the `turtlebot4_node` and `turtlebot4_ignition_hmi_node` required to control the HMI plugin and robot behaviour.

Turtlebot 4 Gazebo launch configuration options:

- **model**: Which TurtleBot 4 model to use
    - options: *standard, lite*
    - default: *standard*
- **rviz**: Whether to launch rviz
    - options: *true, false*
    - default: *false*
- **localization**: Whether to launch localization
    - options: *true, false*
    - default: *false*
- **slam**: Whether to launch SLAM
    - options: *true, false*
    - default: *false*
- **nav2**: Whether to launch Nav2
    - options: *true, false*
    - default: *false*
- **world**: Which world to use for simulation
    - options: *depot, maze, warehouse*
    - default: *warehouse*
- **namespace**: Optional robot namespace
  - options: Any valid ROS 2 name as a string
  - default: blank ("")
- **x**: x coordinate of the robot and dock spawn location in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*
- **y**: y coordinate of the robot and dock spawn location in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*
- **z**: z coordinate of the robot and dock spawn location in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*
- **yaw**: robot and dock orientation at spawn in the gazebo world
  - options: float representing a valid free location in the map
  - default: *0.0*

Running the simulator with default settings:
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

Running synchronous SLAM with Nav2:
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py slam:=true nav2:=true rviz:=true
```

{% endtab %}
{% endtabs %}

## Gazebo GUI Plugins

{% tabs bringup %}
{% tab bringup galactic %}
> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

The `turtlebot4_ignition_gui_plugins` package contains the source code for the TurtleBot 4 HMI GUI plugin.

The [TurtleBot 4 HMI GUI plugin](https://github.com/turtlebot/turtlebot4_simulator/tree/galactic/turtlebot4_ignition_gui_plugins/Turtlebot4Hmi) is only used for the standard model. The lite model uses the [Create® 3 HMI GUI plugin](https://github.com/iRobotEducation/create3_sim/tree/main/irobot_create_ignition/irobot_create_ignition_plugins/Create3Hmi).

{% endtab %}
{% tab bringup humble %}
The `turtlebot4_ignition_gui_plugins` package contains the source code for the TurtleBot 4 HMI GUI plugin.

The [TurtleBot 4 HMI GUI plugin](https://github.com/turtlebot/turtlebot4_simulator/tree/humble/turtlebot4_ignition_gui_plugins/Turtlebot4Hmi) is only used for the standard model. The lite model uses the [Create® 3 HMI GUI plugin](https://github.com/iRobotEducation/create3_sim/tree/main/irobot_create_ignition/irobot_create_ignition_plugins/Create3Hmi).

{% endtab %}
{% tab bringup jazzy %}
The `turtlebot4_gz_gui_plugins` package contains the source code for the TurtleBot 4 HMI GUI plugin.

The [TurtleBot 4 HMI GUI plugin](https://github.com/turtlebot/turtlebot4_simulator/tree/jazzy/turtlebot4_gz_gui_plugins/Turtlebot4Hmi) is only used for the standard model. The lite model uses the [Create® 3 HMI GUI plugin](https://github.com/iRobotEducation/create3_sim/tree/jazzy/irobot_create_gz/irobot_create_gz_plugins/Create3Hmi).

{% endtab %}
{% endtabs %}

<figure class="aligncenter">
    <img src="media/turtlebot4_hmi_gui.png" alt="TurtleBot 4 HMI GUI" style="width: 35%"/>
    <figcaption>TurtleBot 4 HMI GUI plugin</figcaption>
</figure>

## Gazebo Toolbox

{% tabs toolbox %}
{% tab toolbox galactic %}
> :warning: **ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release

The `turtlebot4_ignition_toolbox` package contains the source code for the TurtleBot 4 HMI node. The TurtleBot 4 HMI node acts as a bridge between the `turtlebot4_node` and `ros_ign_bridge` to convert the custom [TurtleBot 4 messages](./turtlebot4_common.md#messages) into standard messages such as `Int32` and `String`.
{% endtab %}
{% tab toolbox humble %}
The `turtlebot4_ignition_toolbox` package contains the source code for the TurtleBot 4 HMI node. The TurtleBot 4 HMI node acts as a bridge between the `turtlebot4_node` and `ros_ign_bridge` to convert the custom [TurtleBot 4 messages](./turtlebot4_common.md#messages) into standard messages such as `Int32` and `String`.
{% endtab %}
{% tab toolbox jazzy %}
The `turtlebot4_gz_toolbox` package contains the source code for the TurtleBot 4 HMI node. The TurtleBot 4 HMI node acts as a bridge between the `turtlebot4_node` and `ros_gz_bridge` to convert the custom [TurtleBot 4 messages](./turtlebot4_common.md#messages) into standard messages such as `Int32` and `String`.
{% endtab %}
{% endtabs %}