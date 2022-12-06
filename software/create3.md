---
sort: 6
---

# Create 3

The TurtleBot 4 can also use all of the actions, messages, and services that the iRobot® Create® 3 platform supports:

**Actions**
<ul class="list-items">
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/AudioNoteSequence.action">AudioNoteSequence</a>: Play a given set of notes from the speaker for a given number of iterations.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DockServo.action">DockServo</a>: Command the robot to dock into its charging station.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DriveArc.action">DriveArc</a>: Command the robot to drive along an arc defined by radius.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/DriveDistance.action">DriveDistance</a>: Command the robot to drive a defined distance in a straight line.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/LedAnimation.action">LedAnimation</a>: Command the lights to perform specified animation.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/NavigateToPosition.action">NavigateToPosition</a>: Command the robot to drive to a goal odometry position using simple approach that rotates to face goal position then translates to goal position then optionally rotates to goal heading.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/RotateAngle.action">RotateAngle</a>: Command the robot to rotate in place a specified amount.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/Undock.action">Undock</a>: Command the robot to undock from its charging station.
    </li>
    <li>
        <a href="https://github.com/iRobotEducation/irobot_create_msgs/blob/main/action/WallFollow.action">WallFollow</a>: Command the robot to wall follow on left or right side using bump and IR sensors.
    </li>
</ul>

#### Messages
* [AudioNote](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/AudioNote.msg): Command the robot to play a note.
* [AudioNoteVector](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/AudioNoteVector.msg): Command the robot to play a sequence of notes.
* [Button](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/Button.msg): Status for a button.
* [Dock](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/msg/Dock.msg): Information about the robot sensing the its dock charging station.
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

#### Services
* [EStop](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/srv/EStop.srv): Set system EStop on or off, cutting motor power when on and enabling motor power when off.
* [RobotPower](https://github.com/iRobotEducation/irobot_create_msgs/blob/main/srv/RobotPower.srv): Power off robot.

See [irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs) for more details.

```note
When publishing or subscribing to topics, make sure that the [QoS](https://docs.ros.org/en/galactic/Concepts/About-Quality-of-Service-Settings.html) that you use matches that of the topic.
```