---
sort: 1
---

# Overview

The TurtleBot 4 runs on Ubuntu 20.04 LTS (Focal Fossa) and currently only supports [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html). The TurtleBot 4 software is entirely open source under the Apache 2.0 license, and is available on the [TurtleBot Github](https://github.com/turtlebot).

There are 2 main computers that run software used by TurtleBot 4: the onboard Raspberry Pi 4, the Create® 3 onboard processor. The user can also connect to the robot with their own PC to visualise sensor data, configure the robot, and more. Each computer is required to run Ubuntu 20.04 with ROS2 Galactic. 

<figure class="aligncenter">
    <img src="media/computer_connections.png" alt="Computer connections" style="width: 60%"/>
    <figcaption>TurtleBot 4 computer connections</figcaption>
</figure>

## Create® 3

The Create® 3 exposes ROS2 topics, actions, and services over both WiFi and the USB-C cable powering the Raspberry Pi. This gives users access to the battery state, sensor data, docking actions, and more. While the Create® 3 can be used with just the USB-C interface, in order to view the robot model on Rviz or run software such as SLAM or Nav2 from a user PC, the Create® 3 will require a WiFi connection.

## Raspberry Pi 4

The Raspberry Pi 4 on both the TurtleBot 4 and TurtleBot 4 Lite comes preinstalled with Ubuntu 20.04 Server, ROS2 Galactic, and TurtleBot 4 software. The latest TurtleBot 4 images can be found [here](http://download.ros.org/downloads/turtlebot4/). The purpose of the Raspberry Pi 4 is to run the TurtleBot 4 ROS nodes, run sensor ROS nodes, use bluetooth to connect to the TurtleBot 4 controller, access GPIO, and more.

## User PC

The user's PC is used to configure the robot, visualise sensor data, run the TurtleBot 4 simulation, and run additional software. The PC is required to run Ubuntu 20.04 with ROS2 Galactic installed, or to use a [Virtual Machine](https://en.wikipedia.org/wiki/Virtual_machine) running Ubuntu 20.04. A typical laptop or desktop will offer significantly higher processing performance than the Raspberry Pi can, so running applications such as [Nav2](nav2.md) or [SLAM](slam.md) on the PC will provide significant performance improvements.
