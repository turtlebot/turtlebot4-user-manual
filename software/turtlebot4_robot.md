---
sort: 3
---

# TurtleBot 4 Robot

Source code is available [here](https://github.com/turtlebot/turtlebot4_robot).

```note
The `turtlebot4_robot` metapackage can be installed on a Raspberry Pi 4B running Ubuntu Server 20.04 with ROS2 Galactic.
```

## Installation

The `turtlebot4_robot` metapackage is pre-installed on the TurtleBot 4 Raspberry Pi image.

### Source installation

To manually install this metapackage from source, clone the git repository:

{% tabs install %}
{% tab install galactic %}

Clone the repository into your workspace:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_robot.git -b galactic
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
{% tab install humble %}

Clone the repository into your workspace:

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_robot.git  -b humble
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

## Base

The `turtlebot4_base` package contains the source code for the [rclcpp](https://github.com/ros2/rclcpp) node `turtlebot4_base_node` which runs on the physical robot. This node interfaces with the GPIO lines of the Raspberry Pi which allows it to read the state of the buttons, as well as write to the LEDs and display. 

```note
This node is only used on the standard TurtleBot 4 model.
```

Publishers:

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
            <td><b>hmi/buttons</b></td>
            <td><i>turtlebot4_msgs/msg/UserButton</i></td>
            <td>Button states of the TurtleBot 4 HMI</td>
        </tr>
    </tbody>
</table>


Subscribers:

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
            <td>The current information that is to be displayed</td>
        </tr>
    </tbody>
    <tbody>
        <tr>
            <td><b>hmi/led/_[led]</b></td>
            <td><i>std_msgs/msg/Int32</i></td>
            <td>Hidden topics indicating the state of each LED</td>
        </tr>
    </tbody>
</table>


### GPIO Interface

The TurtleBot 4 uses *libgpiod* to interface with the GPIO lines of the Raspberry Pi. The `gpiochip0` device represents the 40-pin header of the Raspberry Pi and is used for reading and writing to these pins.

### I2C Interface

The linux I2C drivers are used to read and write data on the I2C buses of the Raspberry Pi. The display's SSD1306 driver is connected to the `i2c-3` device by default, but other buses are available too.

### SSD1306

The SSD1306 is a driver for OLED displays. It receives commands over a communication bus (I2C for the TurtleBot 4) and controls how the physical display behaves. The TurtleBot 4 uses a modified version of this [STM32 SSD1306 driver](https://github.com/afiskon/stm32-ssd1306) to write pixels, shapes and characters to the display.

### Configuration

```warning
Do NOT change pin definitions if you are using the standard PCBA or do not know what you are doing.
```

The `turtlebot4_base_node` pin definitions can be set with ROS parameters. The default configuration is:

```yaml
turtlebot4_base_node:
  ros__parameters:
    # GPIO definition for HMI. Do NOT change if you are using the standard PCBA.
    gpio:
      user_button_1: 13
      user_button_2: 19
      user_button_3: 16
      user_button_4: 26

      led_green_power: 17
      led_green_motors: 18
      led_green_comms: 27
      led_green_wifi: 24
      led_green_battery: 22
      led_red_battery: 23
      led_green_user_1: 25
      led_green_user_2: 6
      led_red_user_2: 12

      display_reset: 2
```

```note
The value for each GPIO device is the GPIO number, NOT the pin number.
```

## Bringup

The `turtlebot4_bringup` package contains the launch and configuration files to run the robots software.

Launch files:
* [Joy Teleop](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/joy_teleop.launch.py): Launches nodes to enable the bluetooth controller.
* [OAKD](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/oakd.launch.py): Launches the OAK-D nodes.
* [RPLIDAR](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/rplidar.launch.py): Launches the RPLIDAR node.
* [Robot](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/robot.launch.py): Launches the TurtleBot 4 nodes.
* [Lite](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/lite.launch.py): Launches all necessary nodes for the TurtleBot 4 Lite.
* [Standard](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/launch/standard.launch.py): Launches all necessary nodes for the TurtleBot 4.

Config files:
* [TurtleBot 4 Controller](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/config/turtlebot4_controller.config.yaml): Configurations for the TurtleBot 4 controller.
* [TurtleBot 4](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_bringup/config/turtlebot4.yaml): Configurations for the `turtlebot4_node` and `turtlebot4_base_node`.

### Robot Upstart

The robot uses the [robot_upstart](https://github.com/clearpathrobotics/robot_upstart/tree/foxy-devel) package to install the bringup launch files as a background service that launches when the robot starts. The launch files are located under the `turtlebot4_bringup` package.

{% tabs upstart %}
{% tab upstart galactic %}

Robot upstart commands:

<table>
    <thead>
        <tr>
            <th>Command</th>
            <th>Bash</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>Stop</b></td>
            <td><code>sudo systemctl stop turtlebot4</code></td>
            <td>Kill all nodes launched by the service. <br/> The service will start again on reboot if it is not uninstalled.</td>
        </tr>
        <tr>
            <td><b>Start</b></td>
            <td><code>sudo systemctl start turtlebot4</code></td>
            <td>Launch all nodes if the service is inactive</td>
        </tr>
        <tr>
            <td><b>Restart</b></td>
            <td><code>sudo systemctl restart turtlebot4</code></td>
            <td>Kill all nodes launched by the service, then launch again</td>
        </tr>
        <tr>
            <td><b>Install</b></td>
            <td>
                <code>install.py model [ROS_DOMAIN_ID]</code> (v0.1.2)<br/> 
                <code>install.py model --domain [ROS_DOMAIN_ID]</code> (v0.1.3)
            </td>
            <td>Install the service. Optionally, set the ROS_DOMAIN_ID.</td>
        </tr>
        <tr>
            <td><b>Uninstall</b></td>
            <td>
                <code>uninstall.py</code>
            </td>
            <td>Uninstall the service. The nodes will no longer be launched on boot.</td>
        </tr>
        <tr>
            <td><b>Status</b></td>
            <td>
                <code>sudo systemctl status turtlebot4</code>
            </td>
            <td>View the current status of the service, and recent logs.</td>
        </tr>
        <tr>
            <td><b>View Logs</b></td>
            <td>
                <code>sudo journalctl -u turtlebot4 -r</code>
            </td>
            <td>View the latest logs from the service.</td>
        </tr>
    </tbody>
</table>

{% endtab %}
{% tab upstart humble %}

TODO: Turtlebot4_setup
{% endtab %}
{% endtabs %}

## Diagnostics

The `turtlebot4_diagnostics` packages contains the source code and launch files for the TurtleBot 4 diagnostics updater.

Launch files:

- **Diagnostics**: Launches the turtlebot4 diagnostics updater and the diagnostic aggregator node.

### Diagnostics Updater

The [diagnostics updater](https://github.com/turtlebot/turtlebot4_robot/blob/galactic/turtlebot4_diagnostics/turtlebot4_diagnostics/diagnostics_updater.py) is a Python3 node that runs on the robot. It subscribes to diagnostic topics records statistics specific to each topic. The diagnostic data is viewable with `rqt_robot_monitor`.

{% tabs diagnostics %}
{% tab diagnostics galactic %}

Diagnostic topics:

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
            <td>Battery voltage and percentage</td>
        </tr>
        <tr>
            <td><b>color/preview/image</b></td>
            <td><i>sensor_msgs/msg/Image</i></td>
            <td>OAK-D color camera data</td>
        </tr>
        <tr>
            <td><b>dock</b></td>
            <td><i>irobot_create_msgs/msg/Dock</i></td>
            <td>Dock status</td>
        </tr>
        <tr>
            <td><b>hazard_detection</b></td>
            <td><i>irobot_create_msgs/msg/HazardDetectionVector</i></td>
            <td>Create 3 Hazards</td>
        </tr>
        <tr>
            <td><b>imu</b></td>
            <td><i>sensor_msgs/msg/Imu</i></td>
            <td>IMU data</td>
        </tr>
        <tr>
            <td><b>mouse</b></td>
            <td><i>irobot_create_msgs/msg/Mouse</i></td>
            <td>Mouse sensor data</td>
        </tr>
        <tr>
            <td><b>scan</b></td>
            <td><i>sensor_msgs/msg/LaserScan</i></td>
            <td>RPLIDAR laser scan data</td>
        </tr>
        <tr>
            <td><b>wheel_status</b></td>
            <td><i>irobot_create_msgs/msg/WheelStatus</i></td>
            <td>Wheels enabled status</td>
        </tr>
    </tbody>
</table>

{% endtab %}
{% tab diagnostics humble %}

Diagnostic topics:

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
            <td>Battery voltage and percentage</td>
        </tr>
        <tr>
            <td><b>color/preview/image</b></td>
            <td><i>sensor_msgs/msg/Image</i></td>
            <td>OAK-D color camera data</td>
        </tr>
        <tr>
            <td><b>dock</b></td>
            <td><i>irobot_create_msgs/msg/DockStatus</i></td>
            <td>Dock status</td>
        </tr>
        <tr>
            <td><b>hazard_detection</b></td>
            <td><i>irobot_create_msgs/msg/HazardDetectionVector</i></td>
            <td>Create 3 Hazards</td>
        </tr>
        <tr>
            <td><b>imu</b></td>
            <td><i>sensor_msgs/msg/Imu</i></td>
            <td>IMU data</td>
        </tr>
        <tr>
            <td><b>mouse</b></td>
            <td><i>irobot_create_msgs/msg/Mouse</i></td>
            <td>Mouse sensor data</td>
        </tr>
        <tr>
            <td><b>scan</b></td>
            <td><i>sensor_msgs/msg/LaserScan</i></td>
            <td>RPLIDAR laser scan data</td>
        </tr>
        <tr>
            <td><b>wheel_status</b></td>
            <td><i>irobot_create_msgs/msg/WheelStatus</i></td>
            <td>Wheels enabled status</td>
        </tr>
    </tbody>
</table>

{% endtab %}
{% endtabs %}

Viewing diagnostics:

```bash
ros2 launch turtlebot4_viz view_diagnostics.launch.py
```

<figure class="aligncenter">
    <img src="media/diagnostics.png" alt="rqt_robot_monitor" style="width: 70%"/>
    <figcaption>Diagnostics data captured with rqt_robot_monitor</figcaption>
</figure>

### Tests

The `turtlebot4_tests` packages contains the source code for the TurtleBot 4 system test scripts. These scripts test basic functionality of the robot and are useful for troubleshooting issues.

### ROS Tests

The ROS tests use ROS topics and actions to test various system functionality. Test results are saved to `~/turtlebot4_test_results/Y_m_d-H_M_S` where `Y_m_d-H_M_S` is the date and time of the test. A rosbag is also recorded for the duration of the test and saved to the same location.

Currently supported tests:

- **Light Ring**: Test the Create® 3 light ring
- **Create® 3 Button**: Test the Create® 3 buttons
- **User LED**: Test the HMI LEDs (TurtleBot 4 model only)
- **User Button**: Test the HMI buttons (TurtleBot 4 model only)
- **Display**: Test the HMI display (TurtleBot 4 model only)
- **Dock**: Test the robots ability to undock and dock.

Running the tests:

```bash
ros2 run turtlebot4_tests ros_tests
```

This will launch a CLI menu where the different tests can be run.

Enter the index of the test and hit enter to start the test. Some tests will run automatically while others require user input.

<figure class="aligncenter">
    <img src="media/ros_tests.png" alt="ROS tests" style="width: 25%"/>
    <figcaption>Running the Light Ring test</figcaption>
</figure>