---
sort: 8
---

# Sensors

## RPLIDAR A1M8

### Connecting

The RPLIDAR connects to the TurtleBot 4 with a micro USB to USB-A cable. The sensor does not require high data throughput, so using a USB 2.0 port is sufficient.

{% tabs udev %}
{% tab udev galactic %}

> :warning: **ROS 2 Galactic is no longer supported** Please consider upgrading to a newer release

Once connected, the RPLIDAR should register on the Raspberry PI as a USB device. If the [udev rules](https://github.com/turtlebot/turtlebot4_setup/blob/galactic/udev/turtlebot4.rules) are installed, the RPLIDAR will appear as `/dev/RPLIDAR`. Otherwise it will be `/dev/ttyUSB0`.


{% endtab %}
{% tab udev humble %}

Once connected, the RPLIDAR should register on the Raspberry PI as a USB device. If the [udev rules](https://github.com/turtlebot/turtlebot4_setup/blob/humble/udev/50-turtlebot4.rules) are installed, the RPLIDAR will appear as `/dev/RPLIDAR`. Otherwise it will be `/dev/ttyUSB0`.


{% endtab %}
{% endtabs %}

To check that the USB device exists, use the command

```bash
ls /dev/RPLIDAR
```

If the device exists, the terminal will echo `/dev/RPLIDAR`.

### Installing

{% tabs rplidar %}
{% tab rplidar galactic %}

> :warning: **ROS 2 Galactic is no longer supported** Please consider upgrading to a newer release

The RPLIDAR drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-galactic-rplidar-ros
```

{% endtab %}
{% tab rplidar humble %}

The RPLIDAR drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-humble-rplidar-ros
```

{% endtab %}
{% endtabs %}

### Running

The RPLIDAR drivers run on boot up as part of the TurtleBot4 service on the robot. The following command should only be used on the computer that the lidar is actively plugged into and only if the automatically starting RPLIDAR node has been stopped or disabled.

```bash
ros2 launch turtlebot4_bringup rplidar.launch.py
```

The laserscan will be published to the */scan* topic by default.

### Controlling

The RPLIDAR driver has a default auto standby mode that will start the motor whenever there is at least one subscriber present on the scan topic and will stop the motor whenever there are 0 subscribers present on the scan topic.

```note
There is currently a bug which results in the scan topic only stopping when the subscriber count drops from 1 or more to 0. This means that the motor will run until the scan topic is subscribed to and then unsubscribed from for the first time.
```

To control the motor manually, the auto standby mode must be disabled and then the `start_motor` and `stop_motor` services can be used. This is controlled in the [`rplidar.launch.py` file in the `turtlebot4_robot` repository](https://github.com/turtlebot/turtlebot4_robot/blob/humble/turtlebot4_bringup/launch/rplidar.launch.py).

To use `stop_motor` the service to stop the motor from spinning, call:

```bash
ros2 service call /stop_motor std_srvs/srv/Empty {}
```

This will also stop scans from publishing.

To use the `start_motor` service to start the motor again, call:

```bash
ros2 service call /start_motor std_srvs/srv/Empty {}
```

``` note
If the robot is namesapced, then the service names above have to be modified to include the namespace.
```

## OAK-D

### Connecting

The OAK-D cameras are connected to the Raspberry Pi with a USB-C to USB-A cable. The cameras requires high data throughput so using a USB 3.0 port is highly recommended.

### Installing

{% tabs rplidar %}
{% tab rplidar galactic %}

> :warning: **ROS 2 Galactic is no longer supported** Please consider upgrading to a newer release

The OAK-D drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-galactic-depthai-ros
```

{% endtab %}
{% tab rplidar humble %}

The OAK-D drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-humble-depthai-ros
```

{% endtab %}
{% endtabs %}

### Running

The OAK-D drivers run on boot up as part of the TurtleBot4 service on the robot. The following command should only be used on the computer that the OAK-D camera is actively plugged into and only if the automatically starting OAK-D node has been stopped or disabled.

The default node used by the TurtleBot 4 can be launched:

```bash
ros2 launch turtlebot4_bringup oakd.launch.py
```

Other nodes are available in the DepthAI ROS [repo](https://github.com/luxonis/depthai-ros).

For example:

```bash
ros2 launch depthai_examples mobile_publisher.launch.py
```

AI examples are available on the DepthAI [github](https://github.com/luxonis/depthai-python). To view the images from these examples you will need to ssh into the robot with a `-X` flag.

```bash
ssh ubuntu@192.168.0.15 -X
```

## Create® 3

The Create® 3 comes with several sensors for safety, object detection, and odometry. For more information on the physical location of the sensors, read the Create® 3 [Hardware Overview](https://iroboteducation.github.io/create3_docs/hw/overview/). Hazards detected by the robot are published to the */hazard_detection* topic, although some sensors also have their own individual topics.

When using these sensors with the Discovery Server network configuration, these topics must be enabled in the [`create3_republisher` launch parameters](https://github.com/iRobotEducation/create3_examples/blob/humble/create3_republisher/bringup/params.yaml).

### Cliff

The Create® 3 has 4 cliff sensors located on the front half of the robot. These sensors measure the distance from the robot to the ground, and prevent the robot from falling off of cliffs.

### Bumper

The bumper is used by the Create® 3 to detect objects or walls that the robot has run in to. It can trigger reflexes to recoil from the object, or use the information to follow the wall.

### Wheeldrop

The wheeldrop is the spring on which the Create® 3 wheels sit. When the robot is lifted off of the ground, the spring is decompressed and the wheeldrop hazard is activated.

### IR Proximity

The IR proxmity sensors are located on the front of the bumper and are used for the wall follow action. The sensor data can be viewed on the */ir_intensity* topic.

### Slip and Stall

Wheel slip and stall is also detected by the Create® 3. The status can be viewed on the */slip_status* and */stall_status* topics.

### Kidnap

The robot uses a fusion of sensor data to detect when it has been picked up and "kidnapped". Motors will be disabled in this state, and will re-enable when placed on the ground again. The */kidnap_status* topic shows the current kidnap state.