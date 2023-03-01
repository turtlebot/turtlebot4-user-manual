---
sort: 2
---

# TurtleBot 4 Humble

TurtleBot 4 RPi4 image changelogs. Latest images are available [here](http://download.ros.org/downloads/turtlebot4/).

## v1.0.0

**OS**: Ubuntu Server 22.04.5 LTS

**ROS**: Humble

**Create® 3 Firmware**: H.1.0

### General

- Added `turtlebot4_setup` configuration tool. See [TurtleBot 4 Setup](../software/turtlebot4_setup.md#configuration-tools).
- Using `turtlebot4_setup` as a debian package to install configuration files. This allows us to update files in the future
without creating new RPI4 images.
- Added namespacing support. See [Multiple Robots](../tutorials/multiple_robots.md#namespacing) for more details.
- Added discovery server support. See [Discovery Server](../setup/networking.md#discovery-server) for more details.
- Added support for the updated OAK-D ROS2 driver. Cameras are now easier to configure, and can be turned off and on at runtime.
- Added power saver mode. The robot will automatically turn off the OAK-D and RPLIDAR when docked to charge faster.
    - Added RPLIDAR and OAK-D start/stop functions to manually turn the sensors on/off through the TurtleBot 4 node.
- Added a function calls publisher. Any function used in the TurtleBot 4 node will have its name published to the `function_calls` topic.
This allows users to add custom functions and hook their own nodes into the system.
- Improved `turtlebot4_base` performance on the Standard model by using a variable refresh rate on the display.
- Added TURTLEBOT4_DIAGNOSTICS environment variable to enable or disable launching diagnostics.
- Reworked navigation launch files to have one for each of SLAM, localization, and Nav2.
- Fixed `Invalid frame ID "wheel_drop_right" passed to canTransform argument source_frame - frame does not exist` error

### TurtleBot 4 Packages

<table>
    <thead>
        <tr>
            <th>Package</th>
            <th>Version</th>
            <th>Changes (from Galactic 0.1.3)</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>turtlebot4_base</td>
            <td>1.0.0</td>
            <td>
                - Namespacing support
            </td>
        </tr>
        <tr>
            <td>turtlebot4_bringup</td>
            <td>1.0.0</td>
            <td>
                - Namespacing support <br/>
                - TURTLEBOT4_DIAGNOSTICS envar to enable/disable diagnostics <br/>
                - Using new OAK-D nodes <br/>
                - Added power saver mode to default turtlebot4_node configuration
            </td>
        </tr>
        <tr>
            <td>turtlebot4_description</td>
            <td>1.0.0</td>
            <td>
                - Namespacing support <br/>
                - Static transforms for wheel_drop links <br/>
                - Updated OAK-D link names for new OAK-D nodes
            </td>
        </tr>
        <tr>
            <td>turtlebot4_diagnostics</td>
            <td>1.0.0</td>
            <td>
                - Namespacing support <br/>
                - Updated dock topic and action type
            </td>
        </tr>
        <tr>
            <td>turtlebot4_msgs</td>
            <td>1.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_navigation</td>
            <td>1.0.0</td>
            <td>
                - Namespacing support <br/>
                - Reworked SLAM, localization, and Nav2 launch files
            </td>
        </tr>
        <tr>
            <td>turtlebot4_node</td>
            <td>1.0.0</td>
            <td>
                - Namespacing support <br/>
                - Added function calls publisher <br/>
                - Update display only when required <br/>
                - Added RPLIDAR and OAK-D start and stop functions <br/>
                - Added power saver mode. Stops OAK-D and RPLIDAR when docked <br/>
                - Updated dock topic and action type
            </td>
        </tr>
        <tr>
            <td>turtlebot4_robot</td>
            <td>1.0.0</td>
            <td>-</td>
        </tr>
        <tr>
            <td>turtlebot4_setup</td>
            <td>1.0.0</td>
            <td>
                - Added setup tool to make configurating easier <br/>
                - Now functions as debian package, installing required files on RPi <br/>
                - Added Discovery Server support
            </td>
        </tr>
        <tr>
            <td>turtlebot4_tests</td>
            <td>1.0.0</td>
            <td>
                - Updated dock topic and action type
            </td>
        </tr>
    </tbody>
</table>
