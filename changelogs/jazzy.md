---
sort: 3
---

# TurtleBot 4 Jazzy

TurtleBot 4 RPi4 image changelogs. Latest images are available [here](http://download.ros.org/downloads/turtlebot4/).

## v2.0.0

**OS**: Ubuntu Server 24.04.5 LTS

**ROS**: Jazzy

**Create® 3 Firmware**: I.0.0

### General

- Due to technical limitations, the Create3 firmware only supports FastRTPS or CycloneDDS; changing
  RMW implementations requires installing the correct firwmare version on the Create3
- Updated `install.py` to support Jazzy
- Use `create3_republisher` by default to expose base platform topics from the RPi4
- ROS Control changes to use `TwistStamped` messages for velocity control instead of `Twist`
- Renamed `ignition` and `ign` packages & launch files to use preferred `gz` nomenclature

### TurtleBot 4 Packages

<table>
    <thead>
        <tr>
            <th>Package</th>
            <th>Version</th>
            <th>Changes</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>turtlebot4_base</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_bringup</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_description</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_diagnostics</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_msgs</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_navigation</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_node</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_robot</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_tests</td>
            <td>2.0.0</td>
            <td>-</td>
        </tr>
    </tbody>
</table>
