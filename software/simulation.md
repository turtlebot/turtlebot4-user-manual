---
sort: 10
---

# Simulation

The simulator allows the user to test the robot without the need for a physical robot. It has all of the same functionality as the real robot. The TurtleBot 4 can be simulated using [Gazebo](http://gazebosim.org/) (previously known as Ignition Gazebo). Unlike [Gazebo Classic](https://classic.gazebosim.org/), Gazebo does not natively support ROS. Instead, it has its own transport stack with a similar topic and node implementation. To communicate with ROS, we can use the [ros_ign_bridge](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge). This ROS node translates data from ROS to Ignition, and vice versa.


## Installing Gazebo

{% tabs requirements %}
{% tab requirements galactic %}
Requirements:
- Ubuntu 20.04
- ROS 2 Galactic
{% endtab %}
{% tab requirements humble %}
Requirements:
- Ubuntu 22.04
- ROS 2 Humble
{% endtab %}
{% tab requirements jazzy %}
Requirements:
- Ubuntu 24.04
- ROS 2 Jazzy
{% endtab %}
{% endtabs %}

Recommended:
- PC with dedicated GPU

Follow the installation instructions described [here](./turtlebot4_simulator.md#installation).

## Launching Gazebo

For a full tutorial using Gazebo, see the [Navigation tutorial](../tutorials/navigation.md)

{% tabs ignition %}
{% tab ignition galactic %}
```warning
**ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release
```

The `ignition.launch.py` launch file has several [launch configurations](turtlebot4_simulator.md#ignition-bringup) that allow the user to customize the simulation.

Default TurtleBot 4 launch:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py
```

Ignition Gazebo will launch and spawn the TurtleBot 4 in the default world along with all of the necessary nodes.

<figure class="aligncenter">
    <img src="media/ignition.png" alt="Ignition Gazebo" style="width: 100%"/>
    <figcaption>TurtleBot 4 in Ignition Gazebo</figcaption>
</figure>

TurtleBot 4 Lite launch:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py model:=lite
```

<figure class="aligncenter">
    <img src="media/ignition_lite.png" alt="Ignition Gazebo Lite" style="width: 100%"/>
    <figcaption>TurtleBot 4 Lite in Ignition Gazebo</figcaption>
</figure>

{% endtab %}
{% tab ignition humble %}

The `turtlebot4_ignition.launch.py` launch file has several [launch configurations](turtlebot4_simulator.md#ignition-bringup) that allow the user to customize the simulation.

Default TurtleBot 4 launch:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

Ignition Gazebo will launch and spawn the TurtleBot 4 in the default world along with all of the necessary nodes.

<figure class="aligncenter">
    <img src="media/ignition.png" alt="Ignition Gazebo" style="width: 100%"/>
    <figcaption>TurtleBot 4 in Ignition Gazebo</figcaption>
</figure>

TurtleBot 4 Lite launch:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite
```

<figure class="aligncenter">
    <img src="media/ignition_lite.png" alt="Ignition Gazebo Lite" style="width: 100%"/>
    <figcaption>TurtleBot 4 Lite in Ignition Gazebo</figcaption>
</figure>

{% endtab %}
{% tab ignition jazzy %}

The `turtlebot4_gz.launch.py` launch file has several [launch configurations](turtlebot4_simulator.md#ignition-bringup) that allow the user to customize the simulation.

Default TurtleBot 4 launch:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

Gazebo will launch and spawn the TurtleBot 4 in the default world along with all of the necessary nodes.

<figure class="aligncenter">
    <img src="media/ignition.png" alt="Gazebo" style="width: 100%"/>
    <figcaption>TurtleBot 4 in Gazebo</figcaption>
</figure>

TurtleBot 4 Lite launch:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite
```

<figure class="aligncenter">
    <img src="media/ignition_lite.png" alt="Gazebo Lite" style="width: 100%"/>
    <figcaption>TurtleBot 4 Lite in Gazebo</figcaption>
</figure>

{% endtab %}
{% endtabs %}
