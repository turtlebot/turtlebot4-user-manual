---
sort: 2
---

# Networking

The TurtleBot 4 consists of two computing units: the Create® 3, and the Raspberry Pi. They connect to each other over a USB-C cable, which is used to both power the Raspberry Pi, and establish a ethernet connection between the units. Both units also have Wi-Fi cards. The Create® 3 is able to connect to only 2.4 GHz networks, while the Raspberry Pi can connect to both 2.4 and 5 GHz networks. The user can also use their PC to communicate with the robot over Wi-Fi.

## DDS

ROS 2 has multiple [DDS](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html) vendors, but the TurtleBot 4 only supports [CycloneDDS](https://github.com/ros2/rmw_cyclonedds) and [FastDDS](https://github.com/ros2/rmw_fastrtps) out of the box. In Galactic, CycloneDDS is the default while in Humble, FastDDS is the default. The TurtleBot 4 can switch between the two DDS implementations on both versions of ROS 2.

## Network Configurations

For ROS 2 networking, the TurtleBot 4 can function with two different configurations: Simple Discovery, and Discovery Server.

### Simple Discovery

Simple Discovery is the default protocol that ROS 2 uses. Simple Discovery uses multicasting to allow a host to send packets to multiple recipients on the network simultaneously. In ROS 2, this allows for all devices on the network to automatically discover each others ROS 2 nodes.

To use the TurtleBot 4 in the multicast configuration, both the RPi4 and the Create® 3 should be connected to the same Wi-Fi network. It is recommended that the RPi4 gets connected to a 5GHz network for improved performance, so the network must bridge the 2.4 and 5 GHz bands.

<figure class="aligncenter">
    <img src="media/simple.png" alt="simple_discovery" style="width: 70%"/>
    <figcaption>TurtleBot 4 Simple Discovery configuration</figcaption>
</figure>

### Discovery Server

The FastDDS Discovery Server allows for one device on the network to act as the discovery server, while the rest of the devices become discovery clients. This is akin to the ROS Master in ROS 1. Check out the Discovery Server [documentation](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html) for more details.

With this configuration, the TurtleBot 4 is able to fully function without the Create® 3 being connected to Wi-Fi.

<figure class="aligncenter">
    <img src="media/discovery.png" alt="discovery_server" style="width: 70%"/>
    <figcaption>TurtleBot 4 Discovery Server configuration</figcaption>
</figure>

### Choosing a networking configuration

Each configuration has its own advantages and disadvantages. Simple Discovery makes it easy to interact with the robot from any device on the network. It is the default discovery protocol in ROS 2, so there is no additional setup required. Also, it supports both CycloneDDS and FastDDS. The main disadvantage is that this protocol uses multicasting which can be problematic with some Wi-Fi networks, such as university and corporate. Additionally, the Create® 3 must also be connected to Wi-Fi for a remote PC to be able to interact with the TurtleBot 4, so the network must bridge the 2.4 and 5 GHz bands.

Using a Discovery Server bypasses multicasting issues and allows for the TurtleBot 4 to function off of just the Raspberry Pi Wi-Fi. This means that if configured as an [Access Point](./robot.html#access-point-mode), the TurtleBot 4 does not require an external network. The disadvantages of this configuration is that extra setup is required on the Create® 3, Raspberry Pi, and User PC. Additionally, the Raspberry Pi must route traffic from the User PC to the Create® 3, and vice versa. Discovery Server also requires FastDDS as the middleware.

```note
Discovery Server currently does not support communicating with multiple TurtleBot 4's simultaneously from one computer.
```