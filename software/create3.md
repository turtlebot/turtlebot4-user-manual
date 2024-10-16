---
sort: 7
---

# Create® 3

The TurtleBot 4 can also use all of the actions, messages, and services that the iRobot® Create® 3 platform supports:

{% tabs create3 %}
{% tab create3 galactic %}
```warning
**ROS 2 Galactic is no longer supported.** Please consider upgrading to a newer release
```

**Actions**

* [AudioNoteSequence](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/AudioNoteSequence.action): Play a given set of notes from the speaker for a given number of iterations.
* [DockServo](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/DockServo.action): Command the robot to dock into its charging station.
* [DriveArc](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/DriveArc.action): Command the robot to drive along an arc defined by radius.
* [DriveDistance](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/DriveDistance.action): Command the robot to drive a defined distance in a straight line.
* [LedAnimation](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/LedAnimation.action): Command the lights to perform specified animation.
* [NavigateToPosition](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/NavigateToPosition.action): Command the robot to drive to a goal odometry position using simple approach that rotates to face goal position then translates to goal position then optionally rotates to goal heading.
* [RotateAngle](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/RotateAngle.action): Command the robot to rotate in place a specified amount.
* [Undock](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/Undock.action): Command the robot to undock from its charging station.
* [WallFollow](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/action/WallFollow.action): Command the robot to wall follow on left or right side using bump and IR sensors.

**Messages**

* [AudioNote](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/AudioNote.msg): Command the robot to play a note.
* [AudioNoteVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/AudioNoteVector.msg): Command the robot to play a sequence of notes.
* [Button](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/Button.msg): Status for a button.
* [Dock](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/Dock.msg): Information about the robot sensing the its dock charging station.
* [HazardDetection](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/HazardDetection.msg): An hazard or obstacle detected by the robot.
* [HazardDetectionVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/HazardDetectionVector.msg): All the hazards and obstacles detected by the robot.
* [InterfaceButtons](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/InterfaceButtons.msg): Status of the 3 interface buttons on the Create® robot faceplate.
* [IrIntensity](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/IrIntensity.msg): Reading from an IR intensity sensor.
* [IrIntensityVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/IrIntensityVector.msg): Vector of current IR intensity readings from all sensors.
* [IrOpcode](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/IrOpcode.msg): Opcode detected by the robot IR receivers. Used to detect the dock and virtual walls.
* [KidnapStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/KidnapStatus.msg): Whether the robot has been picked up off the ground.
* [LedColor](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/LedColor.msg): RGB values for an LED.
* [LightringLeds](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/LightringLeds.msg): Command RGB values of 6 lightring lights.
* [Mouse](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/Mouse.msg): Reading from a mouse sensor.
* [SlipStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/SlipStatus.msg): Whether the robot is currently slipping or not.
* [StopStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/StopStatus.msg): Whether the robot is currently stopped or not.
* [WheelStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/WheelStatus.msg): Current/PWM readings from the robot's two wheels in addition to whether wheels are enabled.
* [WheelTicks](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/WheelTicks.msg): Reading from the robot two wheels encoders.
* [WheelVels](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/msg/WheelVels.msg): Indication about the robot two wheels current speed.

**Services**

* [EStop](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/srv/EStop.srv): Set system EStop on or off, cutting motor power when on and enabling motor power when off.
* [RobotPower](https://github.com/iRobotEducation/irobot_create_msgs/blob/galactic/srv/RobotPower.srv): Power off robot.

See [irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs) for more details.

```note
When publishing or subscribing to topics, make sure that the [QoS](https://docs.ros.org/en/galactic/Concepts/About-Quality-of-Service-Settings.html) that you use matches that of the topic.
```

{% endtab %}
{% tab create3 humble %}

```note
When using the Create® 3 with the Discovery Server network configuration, these topics, actions and services must be enabled in the [`create3_republisher` launch parameters](https://github.com/iRobotEducation/create3_examples/blob/humble/create3_republisher/bringup/params.yaml). For more information see the [Create® 3 Republisher section](#create-3-republisher)
```

**Actions**

* [AudioNoteSequence](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/AudioNoteSequence.action): Play a given set of notes from the speaker for a given number of iterations.
* [Dock](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/Dock.action): Command the robot to dock into its charging station.
* [DriveArc](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DriveArc.action): Command the robot to drive along an arc defined by radius.
* [DriveDistance](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DriveDistance.action): Command the robot to drive a defined distance in a straight line.
* [LedAnimation](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/LedAnimation.action): Command the lights to perform specified animation.
* [NavigateToPosition](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/NavigateToPosition.action): Command the robot to drive to a goal odometry position using simple approach that rotates to face goal position then translates to goal position then optionally rotates to goal heading.
* [RotateAngle](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/RotateAngle.action): Command the robot to rotate in place a specified amount.
* [Undock](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/Undock.action): Command the robot to undock from its charging station.
* [WallFollow](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/WallFollow.action): Command the robot to wall follow on left or right side using bump and IR sensors.

**Messages**

* [AudioNote](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/AudioNote.msg): Command the robot to play a note.
* [AudioNoteVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/AudioNoteVector.msg): Command the robot to play a sequence of notes.
* [Button](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/Button.msg): Status for a button.
* [DockStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/DockStatus.msg): Information about the robot sensing the its dock charging station.
* [HazardDetection](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/HazardDetection.msg): An hazard or obstacle detected by the robot.
* [HazardDetectionVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/HazardDetectionVector.msg): All the hazards and obstacles detected by the robot.
* [InterfaceButtons](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/InterfaceButtons.msg): Status of the 3 interface buttons on the Create® robot faceplate.
* [IrIntensity](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/IrIntensity.msg): Reading from an IR intensity sensor.
* [IrIntensityVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/IrIntensityVector.msg): Vector of current IR intensity readings from all sensors.
* [IrOpcode](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/IrOpcode.msg): Opcode detected by the robot IR receivers. Used to detect the dock and virtual walls.
* [KidnapStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/KidnapStatus.msg): Whether the robot has been picked up off the ground.
* [LedColor](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/LedColor.msg): RGB values for an LED.
* [LightringLeds](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/LightringLeds.msg): Command RGB values of 6 lightring lights.
* [Mouse](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/Mouse.msg): Reading from a mouse sensor.
* [SlipStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/SlipStatus.msg): Whether the robot is currently slipping or not.
* [StopStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/StopStatus.msg): Whether the robot is currently stopped or not.
* [WheelStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/WheelStatus.msg): Current/PWM readings from the robot's two wheels in addition to whether wheels are enabled.
* [WheelTicks](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/WheelTicks.msg): Reading from the robot two wheels encoders.
* [WheelVels](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/WheelVels.msg): Indication about the robot two wheels current speed.

**Services**

* [EStop](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/srv/EStop.srv): Set system EStop on or off, cutting motor power when on and enabling motor power when off.
* [RobotPower](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/srv/RobotPower.srv): Power off robot.

See [irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs) for more details.

```note
When publishing or subscribing to topics, make sure that the [QoS](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) that you use matches that of the topic.
```

## Create® 3 Republisher {#create-3-republisher}

In certain network configurations it is beneficial to isolate the Create® 3 from the remainder of the ROS 2 network traffic. The Create® 3 has limited resources and can be overwhelmed with too many topics to discover. The Create® 3 Republisher is used to achieve this. All communication to the Create® 3 is routed through the Raspberry Pi via the republished ROS 2 topics, action and services. This allows nodes to interact with the Create® 3 ROS 2 elements normally while only needing communication with the Raspberry Pi.

This is achieved by having the Create® 3 on its own unique namespace which is a combination of the robot namespace and `_do_not_use`. This results in all of its topics being hidden and indicates clearly that these topics, actions and services are not to be used directly by the user. The Create® 3 only communicates with the single Raspberry Pi that is present on the same robot. All of the Create® 3 topics, actions and services are then republished and relayed by the Raspberry Pi `create3_republisher` node with just the robot namespace so they are fully visible and available to the rest of the ROS 2 network. In order to interact with the Create® 3, any given node will interact with the topics, actions, and servers generated by the Raspberry Pi, and the republisher node on the Raspberry Pi will relay that information back to the Create® 3.

```note
The `_do_not_use` namespace, as the name suggests, should _not_ be used by any other nodes.  The only place this namespace should ever be referenced is inside the configuration file for the `create3_republisher` node and within the Create® 3's application configuration.

All other ROS nodes, shell commands (e.g. `ros2 topic echo ...`) should use the republished topics, actions, and services.
```

In order to prevent overloading the system with too many topics, the number of topics, actions and services that are relayed through the republisher node are limited. The default list of topics, actions and services that are being relayed can be found in the [`create3_republisher` launch parameters](https://github.com/iRobotEducation/create3_examples/blob/humble/create3_republisher/bringup/params.yaml). To optimize the system, update this list to only include the necessary topics, actions and services that are needed.

{% endtab %}
{% tab create3 jazzy %}

```note
When using the Create® 3 with the Discovery Server network configuration, these topics, actions and services must be enabled in the [`create3_republisher` launch parameters](https://github.com/iRobotEducation/create3_examples/blob/jazzy/create3_republisher/bringup/params.yaml). For more information see the [Create® 3 Republisher section](#create-3-republisher)
```

**Actions**

* [AudioNoteSequence](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/AudioNoteSequence.action): Play a given set of notes from the speaker for a given number of iterations.
* [Dock](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/Dock.action): Command the robot to dock into its charging station.
* [DriveArc](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DriveArc.action): Command the robot to drive along an arc defined by radius.
* [DriveDistance](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DriveDistance.action): Command the robot to drive a defined distance in a straight line.
* [LedAnimation](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/LedAnimation.action): Command the lights to perform specified animation.
* [NavigateToPosition](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/NavigateToPosition.action): Command the robot to drive to a goal odometry position using simple approach that rotates to face goal position then translates to goal position then optionally rotates to goal heading.
* [RotateAngle](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/RotateAngle.action): Command the robot to rotate in place a specified amount.
* [Undock](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/Undock.action): Command the robot to undock from its charging station.
* [WallFollow](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/WallFollow.action): Command the robot to wall follow on left or right side using bump and IR sensors.

**Messages**

* [AudioNote](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/AudioNote.msg): Command the robot to play a note.
* [AudioNoteVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/AudioNoteVector.msg): Command the robot to play a sequence of notes.
* [Button](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/Button.msg): Status for a button.
* [DockStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/DockStatus.msg): Information about the robot sensing the its dock charging station.
* [HazardDetection](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/HazardDetection.msg): An hazard or obstacle detected by the robot.
* [HazardDetectionVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/HazardDetectionVector.msg): All the hazards and obstacles detected by the robot.
* [InterfaceButtons](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/InterfaceButtons.msg): Status of the 3 interface buttons on the Create® robot faceplate.
* [IrIntensity](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/IrIntensity.msg): Reading from an IR intensity sensor.
* [IrIntensityVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/IrIntensityVector.msg): Vector of current IR intensity readings from all sensors.
* [IrOpcode](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/IrOpcode.msg): Opcode detected by the robot IR receivers. Used to detect the dock and virtual walls.
* [KidnapStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/KidnapStatus.msg): Whether the robot has been picked up off the ground.
* [LedColor](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/LedColor.msg): RGB values for an LED.
* [LightringLeds](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/LightringLeds.msg): Command RGB values of 6 lightring lights.
* [Mouse](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/Mouse.msg): Reading from a mouse sensor.
* [SlipStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/SlipStatus.msg): Whether the robot is currently slipping or not.
* [StopStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/StopStatus.msg): Whether the robot is currently stopped or not.
* [WheelStatus](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/WheelStatus.msg): Current/PWM readings from the robot's two wheels in addition to whether wheels are enabled.
* [WheelTicks](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/WheelTicks.msg): Reading from the robot two wheels encoders.
* [WheelVels](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/WheelVels.msg): Indication about the robot two wheels current speed.

**Services**

* [EStop](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/srv/EStop.srv): Set system EStop on or off, cutting motor power when on and enabling motor power when off.
* [RobotPower](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/srv/RobotPower.srv): Power off robot.

See [irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs) for more details.

```note
When publishing or subscribing to topics, make sure that the [QoS](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html) that you use matches that of the topic.
```

## Create® 3 Republisher {#create-3-republisher}

By default, the Turtlebot 4 uses the `create3_republisher` node to isolate the Create® 3 from the remainder of the ROS 2 network traffic.  This helps limit the network traffic that reaches the Create® 3 when it is connected to the wireless network, especially when multiple robots are connected.  All ROS 2 traffic is routed via the Turtlebot 4's Raspberry Pi computer, and the `create3_republisher` node exposes the Create® 3's topics, services, and actions the robot's main namespace.  This allows external workstations and other robots to interact with the robot via the Raspberry Pi's wireless connection.

The Create® 3's topics, services, and actions are all available inside a namespace called `_do_not_use`.  This results in the topics being hidden, and clearly indicates that they should not be used directly by the user. The Create® 3 only communicates with the single Raspberry Pi that is present on the same robot. All of the Create® 3 topics, actions and services are then republished and relayed by the Raspberry Pi `create3_republisher` node with just the robot namespace so they are fully visible and available to the rest of the ROS 2 network. In order to interact with the Create® 3, any given node will interact with the topics, actions, and servers generated by the Raspberry Pi, and the republisher node on the Raspberry Pi will relay that information back to the Create® 3.

```note
The `_do_not_use` namespace, as the name suggests, should _not_ be used by any other nodes.  The only place this namespace should ever be referenced is inside the configuration file for the `create3_republisher` node and within the Create® 3's application configuration.

All other ROS nodes, shell commands (e.g. `ros2 topic echo ...`) should use the republished topics, actions, and services.
```

In order to prevent overloading the system with too many topics, the number of topics, actions and services that are relayed through the republisher node are limited. The default list of topics, actions and services that are being relayed can be found in the [`create3_republisher` launch parameters](https://github.com/iRobotEducation/create3_examples/blob/jazzy/create3_republisher/bringup/params.yaml). To optimize the system, update this list to only include the necessary topics, actions and services that are needed.

{% endtab %}
{% endtabs %}