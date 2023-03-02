---
sort: 5
---

# TurtleBot 4 Desktop

The `turtlebot4_desktop` metapackage contains packages used for visualising and interfacing with the TurtleBot 4 from a PC.

## Installation

Source code is available [here](https://github.com/turtlebot/turtlebot4_desktop).

```note
The `turtlebot4_desktop` metapackage can be installed on a PC running Ubuntu Desktop 20.04 with ROS 2 Galactic.
```

### Debian installation

{% tabs debian %}
{% tab debian galactic %}

To install the metapackage through apt:

```bash
sudo apt update
sudo apt install ros-galactic-turtlebot4-desktop
```

{% endtab %}
{% tab debian humble %}

To install the metapackage through apt:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-desktop
```

{% endtab %}
{% endtabs %}

### Source installation

{% tabs source %}
{% tab source galactic %}

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b galactic
```

Install dependencies:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```

Build the packages:

```bash
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
```

{% endtab %}
{% tab source humble %}

To manually install this metapackage from source, clone the git repository:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_desktop.git -b humble
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

{% endtab %}
{% endtabs %}

## Visualisation

The `turtlebot4_viz` package contains launch files and configurations for viewing the robot in Rviz2, and viewing the diagnostics.

{% tabs debian %}
{% tab debian galactic %}

Launch files:
* [View Diagnostics](https://github.com/turtlebot/turtlebot4_desktop/blob/galactic/turtlebot4_viz/launch/view_diagnostics.launch.py): Launches `rqt_robot_monitor` to view diagnostic data.
* [View Model](https://github.com/turtlebot/turtlebot4_desktop/blob/galactic/turtlebot4_viz/launch/view_model.launch.py): Launches `rviz2`. Used to view the model and sensor data.
* [View Robot](https://github.com/turtlebot/turtlebot4_desktop/blob/galactic/turtlebot4_viz/launch/view_robot.launch.py): Launches `rviz2`. Used to view the robot while navigating.

Viewing the robot while mapping:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

Viewing diagnostics:

```bash
ros2 launch turtlebot4_viz view_diagnostics.launch.py
```

{% endtab %}
{% tab debian humble %}

Launch files:
* [View Diagnostics](https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/launch/view_diagnostics.launch.py): Launches `rqt_robot_monitor` to view diagnostic data.
* [View Model](https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/launch/view_model.launch.py): Launches `rviz2`. Used to view the model and sensor data.
* [View Robot](https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/launch/view_robot.launch.py): Launches `rviz2`. Used to view the robot while navigating.

Viewing the robot while mapping:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

Viewing diagnostics:

```bash
ros2 launch turtlebot4_viz view_diagnostics.launch.py
```

If your robot is using a namespace, use it as a launch argument:

```bash
ros2 launch turtlebot4_viz view_model.launch.py namespace:=/my_robot_namespace
```

{% endtab %}
{% endtabs %}


