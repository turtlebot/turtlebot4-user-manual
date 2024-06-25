---
sort: 1
---

# Overview

The TurtleBot 4 runs on Ubuntu 20.04 LTS (Focal Fossa) or Ubuntu 22.04 (Jammy Jellyfish) and currently only supports [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html) or [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) respectively. The TurtleBot 4 software is entirely open source under the Apache 2.0 license, and is available on the [TurtleBot Github](https://github.com/turtlebot).

There are 2 main computers that run software used by TurtleBot 4: the onboard Raspberry Pi 4B, and the Create® 3 onboard processor. The user can also connect to the robot with their own PC to visualise sensor data, configure and control the robot, and more. Each computer is required to run the same version of Ubuntu and ROS 2 as the Turtlebot 4. 

<figure class="aligncenter">
    <img src="media/computer_connections.png" alt="Computer connections" style="width: 60%"/>
    <figcaption>TurtleBot 4 computer connections (in Simple Discovery mode)</figcaption>
</figure>

## Create® 3

The Create® 3 exposes ROS 2 topics, actions, and services over both WiFi and the USB-C cable powering the Raspberry Pi. This gives users access to the battery state, sensor data, docking actions, and more. While the Create® 3 can be used with just the USB-C interface, when used with the simple discovery networking configuration, the Create® 3 will require a WiFi connection in order to view the robot model on Rviz or run software such as SLAM or Nav2 from a user computer.

## Raspberry Pi 4B

The Raspberry Pi 4B on both the TurtleBot 4 and TurtleBot 4 Lite comes preinstalled with Ubuntu 22.04 Server, ROS 2 Humble, and TurtleBot 4 software. The latest TurtleBot 4 images can be found [here](http://download.ros.org/downloads/turtlebot4/). The purpose of the Raspberry Pi 4B is to run the TurtleBot 4 ROS nodes, run sensor ROS nodes, use Bluetooth to connect to the TurtleBot 4 controller, access GPIO, and more.

## User PC

The user's PC is used to configure the robot, visualise sensor data, run the TurtleBot 4 simulation, and run additional software. The PC is required to run Ubuntu 20.04 with ROS 2 Galactic installed, Ubuntu 22.04 with Ros 2 Humble installed or to use a [Virtual Machine](https://en.wikipedia.org/wiki/Virtual_machine) running Ubuntu 20.04 or 22.04. This version must match the version running on the robot. A typical laptop or desktop will offer significantly higher processing performance than the Raspberry Pi can, so running applications such as [Nav2 or SLAM](./turtlebot4_common.md#navigation) on the PC will provide significant performance improvements.
