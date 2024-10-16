---
sort: 8
---

# Sensors

## RPLIDAR A1M8

### Connecting

The RPLIDAR connects to the TurtleBot 4 with a micro USB to USB-A cable. The sensor does not require high data throughput, so using a USB 2.0 port is sufficient.

Once connected, the RPLIDAR should register on the Raspberry PI as a USB device. If the [udev rules](https://github.com/turtlebot/turtlebot4_setup/blob/jazzy/udev/turtlebot4.rules) are installed, the RPLIDAR will appear as `/dev/RPLIDAR`. Otherwise it will be `/dev/ttyUSB0`.

To check that the USB device exists, use the command

```bash
ls -l /dev/RPLIDAR
```

If the device exists, the command will print something of the form
```
lrwxrwxrwx 1 root root 7 Oct 10 13:49 /dev/RPLIDAR -> ttyUSB0
```
If the device does not exist, the command will print
```
ls: cannot access '/dev/RPLIDAR': No such file or directory
```

### Installing

The RPLIDAR drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-${ROS_DISTRO}-rplidar-ros
```

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

The OAK-D drivers are installed by default on all TurtleBot 4's. To manually install, run:

```bash
sudo apt install ros-${ROS_DISTRO}-depthai-ros
```

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

AI examples are available on the DepthAI [github](https://github.com/luxonis/depthai-python). To view the images from these examples you will need to ssh into the robot with a `-X` flag:
```bash
ssh -X ubuntu@10.42.0.1
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

The IR proxmity sensors are located on the front of the bumper and are used for the wall follow action. The sensor data can be viewed on the `/ir_intensity` topic.

### Slip and Stall

Wheel slip and stall is also detected by the Create® 3. The status can be viewed on the `/slip_status` and `/stall_status` topics.

### Kidnap

The robot uses a fusion of sensor data to detect when it has been picked up and "kidnapped". Motors will be disabled in this state, and will re-enable when placed on the ground again. The `/kidnap_status` topic shows the current kidnap state.