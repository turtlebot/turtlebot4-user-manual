---
sort: 6
---

# TurtleBot 4 Navigator

{% tabs navigation %}
{% tab navigation galactic %}

The [TurtleBot 4 Navigator](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/turtlebot4_navigation/turtlebot4_navigator.py) is a Python node that adds on to the [Nav2 Simple Commander](https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/robot_navigator.py). It includes TurtleBot 4 specific features such as docking and undocking, as well as easy to use methods for navigating.

{% endtab %}
{% tab navigation humble %}

The [TurtleBot 4 Navigator](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/turtlebot4_navigation/turtlebot4_navigator.py) is a Python node that adds on to the [Nav2 Simple Commander](https://github.com/ros-planning/navigation2/blob/humble/nav2_simple_commander/nav2_simple_commander/robot_navigator.py). It includes TurtleBot 4 specific features such as docking and undocking, as well as easy to use methods for navigating.

{% endtab %}
{% endtabs %}

```note
TurtleBot 4 Navigator requires at least version 1.0.11 of Nav2 Simple Commander
```

## Install Tutorial Package

The following examples can be installed with `sudo apt install ros-$ROS_DISTRO-turtlebot4-tutorials` and are available at <https://github.com/turtlebot/turtlebot4_tutorials>. For each example, the robot starts on a dock at the origin of the map.

```note
All of these examples are designed to be run in the simulation environment. In order to run these tutorial codes with a physical robot you must install the tutorial package from source and modify target destinations to ones that make sense in your map. You then must individually launch SLAM, Nav2, Localization, and RViz as necessary and pass a map of your environment when launching localization. This is only recommended for intermediate users and up because there are no walkthrough instructions.
```

## Navigate to Pose

This example demonstrates the same behaviour as [Nav2 Goal](./navigation.md/#nav2-goal). The Nav2 stack is given a pose on the map with which it calculates a path. The robot then attempts to drive along the path. This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

{% tabs navigation %}
{% tab navigation galactic %}

To run this example, start the Ignition Gazebo simulation:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav:=true slam:=off localization:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials nav_to_pose
```

{% endtab %}
{% tab navigation humble %}

To run this example, start the Ignition Gazebo simulation:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials nav_to_pose
```

{% endtab %}
{% endtabs %}

### Code breakdown


{% tabs navigation %}
{% tab navigation galactic %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/nav_to_pose.py).


Let's take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = navigator.getPoseStamped([13.0, 5.0], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    rclpy.shutdown()
```

{% endtab %}
{% tab navigation humble %}


The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/humble/turtlebot4_python_tutorials/turtlebot4_python_tutorials/nav_to_pose.py).


Let's take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    rclpy.shutdown()
```

{% endtab %}
{% endtabs %}


#### Initialise the node

We start by initialising `rclpy` and creating the `TurtleBot4Navigator` object. This will initialise any ROS 2 publishers, subscribers and action clients that we need.

```py
rclpy.init()

navigator = TurtleBot4Navigator()
```

#### Dock the robot

Next, we check if the robot is docked. If it is not, we send an action goal to dock the robot. By docking the robot we guarantee that it is at the [0.0, 0.0] coordinates on the map.

```py
if not navigator.getDockedStatus():
    navigator.info('Docking before initialising pose')
    navigator.dock()
```

#### Set the initial pose

Now that we know the robot is docked, we can set the initial pose to [0.0, 0.0], facing "North".

```py
initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
navigator.setInitialPose(initial_pose)
```

The TurtleBot 4 Navigator uses cardinal directions to set the orientation of the robot relative to the map. You can use actual integers or floating points if you need a more precise direction.

```py
class TurtleBot4Directions(IntEnum):
    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315
```

```note
These cardinal directions are relative to the map, not the actual magnetic north pole. Driving north is equivalent to driving upwards on the map, west is driving left, and so on.
```

#### Wait for Nav2

Once the initial position has been set, the Nav2 stack will place the robot at that position on the map and begin localizing. We want to wait for Nav2 to be ready before we start sending navigation goals.

```py
navigator.waitUntilNav2Active()
```

```note
This call will block until Nav2 is ready. Make sure you have launched Nav2.
```

#### Set the goal pose

{% tabs navigation %}
{% tab navigation galactic %}

Now we can create a [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/galactic/geometry_msgs/msg/PoseStamped.msg) message. The `getPoseStamped` method makes it easy for us. All we have to do is pass in a list describing the x and y position that we want to drive to on the map, and the direction that we want the robot to be facing when it reaches that point.

```py
goal_pose = navigator.getPoseStamped([13.0, 5.0], TurtleBot4Directions.EAST)
```

{% endtab %}
{% tab navigation humble %}

Now we can create a [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/PoseStamped.msg) message. The `getPoseStamped` method makes it easy for us. All we have to do is pass in a list describing the x and y position that we want to drive to on the map, and the direction that we want the robot to be facing when it reaches that point.

```py
goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
```

{% endtab %}
{% endtabs %}
#### Undock the robot and go to the goal pose

We are ready to drive to the goal pose. We start by undocking the robot so that it does not attempt to drive through the dock, and then send the goal pose. As the robot drives to the goal pose, we will be receiving feedback from the action. This feedback includes the estimated time of arrival.

```py
navigator.undock()

navigator.startToPose(goal_pose)
```

Once the robot has reached the goal, we call `rclpy.shutdown()` to gracefully destroy the rclpy context.

### Watch navigation progress in Rviz

You can visualise the navigation process in Rviz. If Rviz wasn't already launched, launch it by calling:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

{% tabs navigation %}
{% tab navigation galactic %}

<figure class="aligncenter">
    <img src="media/nav_to_pose_rviz.gif" alt="Nav to pose" style="width: 100%"/>
    <figcaption>Navigate to a pose</figcaption>
</figure>

{% endtab %}
{% tab navigation humble %}

<figure class="aligncenter">
    <video style="width: 100%" controls autoplay muted>
    <source src="media/nav_to_pose_rviz_humble.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
    <figcaption>Navigate to a pose (3x speed)</figcaption>
</figure>

{% endtab %}
{% endtabs %}

## Navigate Through Poses

{% tabs navigation %}
{% tab navigation galactic %}

This example demonstrates the [Navigate Through Poses](https://navigation.ros.org/behavior_trees/trees/nav_through_poses_recovery.html) behaviour tree. The Nav2 stack is given a set of poses on the map and creates a path that goes through each pose in order until the last pose is reached. The robot then attempts to drive along the path. This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

To run this example, start the Ignition Gazebo simulation:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav:=true slam:=off localization:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials nav_through_poses
```

{% endtab %}
{% tab navigation humble %}

This example demonstrates the [Navigate Through Poses](https://navigation.ros.org/behavior_trees/trees/nav_through_poses_recovery.html) behaviour tree. The Nav2 stack is given a set of poses on the map and creates a path that goes through each pose in order until the last pose is reached. The robot then attempts to drive along the path. This example is demonstrated in the `warehouse` world of the TurtleBot 4 simulation.

To run this example, start the Ignition Gazebo simulation:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials nav_through_poses
```

{% endtab %}
{% endtabs %}

### Code breakdown

{% tabs navigation %}
{% tab navigation galactic %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/nav_through_poses.py).


Let's take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([0.0, -1.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([1.7, -1.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([1.6, -3.5], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([6.75, -3.46], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([7.4, -1.0], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-1.0, -1.0], TurtleBot4Directions.WEST))

    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()
```


{% endtab %}
{% tab navigation humble %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/humble/turtlebot4_python_tutorials/turtlebot4_python_tutorials/nav_through_poses.py).


Let's take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.0, -0.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-3.0, -3.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([3.0, -3.0], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([9.0, -1.0], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))

    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()
```

{% endtab %}
{% endtabs %}

This example starts the same as [navigate to pose](#navigate-to-pose). We initialise the node, make sure the robot is docked, and set the initial pose. Then we wait for Nav2 to become active.

#### Set goal poses

The next step is to create a list of `PoseStamped` messages which represent the poses that the robot needs to drive through.

{% tabs navigation %}
{% tab navigation galactic %}

```py
goal_pose = []
goal_pose.append(navigator.getPoseStamped([0.0, -1.0], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([1.7, -1.0], TurtleBot4Directions.EAST))
goal_pose.append(navigator.getPoseStamped([1.6, -3.5], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([6.75, -3.46], TurtleBot4Directions.NORTH_WEST))
goal_pose.append(navigator.getPoseStamped([7.4, -1.0], TurtleBot4Directions.SOUTH))
goal_pose.append(navigator.getPoseStamped([-1.0, -1.0], TurtleBot4Directions.WEST))
```
{% endtab %}
{% tab navigation humble %}

```py
goal_pose = []
goal_pose.append(navigator.getPoseStamped([-3.0, -0.0], TurtleBot4Directions.EAST))
goal_pose.append(navigator.getPoseStamped([-3.0, -3.0], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([3.0, -3.0], TurtleBot4Directions.NORTH_WEST))
goal_pose.append(navigator.getPoseStamped([9.0, -1.0], TurtleBot4Directions.WEST))
goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))
```

{% endtab %}
{% endtabs %}

#### Navigate through the poses

Now we can undock the robot and begin navigating through each point. Once the robot has reached the final pose, it will then return to the dock.

```py
navigator.undock()

navigator.startThroughPoses(goal_pose)

navigator.dock()
```

### Watch navigation progress in Rviz

You can visualise the navigation process in Rviz. If Rviz wasn't already launched, launch it by calling:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
{% tabs navigation %}
{% tab navigation galactic %}

<figure class="aligncenter">
    <img src="media/nav_through_pose_rviz.gif" alt="Nav through pose" style="width: 100%"/>
    <figcaption>Navigate through a set of poses</figcaption>
</figure>

{% endtab %}
{% tab navigation humble %}

<figure class="aligncenter">
    <video style="width: 100%" controls autoplay muted>
    <source src="media/nav_through_pose_rviz_humble.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
    <figcaption>Navigate through a set of poses</figcaption>
</figure>

{% endtab %}
{% endtabs %}

## Follow Waypoints

{% tabs navigation %}
{% tab navigation galactic %}

This example demonstrates how to follow waypoints. The Nav2 stack is given a set of waypoints on the map and creates a path that goes through each waypoint in order until the last waypoint is reached. The robot then attempts to drive along the path. The difference between this example and Navigating Through Poses is that when following waypoints the robot will plan to reach each waypoint individually, rather than planning to reach the last pose by driving through the other poses.  This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

To run this example, start the Ignition Gazebo simulation:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav:=true slam:=off localization:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials follow_waypoints
```

{% endtab %}
{% tab navigation humble %}

This example demonstrates how to follow waypoints. The Nav2 stack is given a set of waypoints on the map and creates a path that goes through each waypoint in order until the last waypoint is reached. The robot then attempts to drive along the path. The difference between this example and Navigating Through Poses is that when following waypoints the robot will plan to reach each waypoint individually, rather than planning to reach the last pose by driving through the other poses.  This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

To run this example, start the Ignition Gazebo simulation:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials follow_waypoints
```

{% endtab %}
{% endtabs %}


### Code breakdown

{% tabs navigation %}
{% tab navigation galactic %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/follow_waypoints.py).

{% endtab %}
{% tab navigation humble %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/humble/turtlebot4_python_tutorials/turtlebot4_python_tutorials/follow_waypoints.py).

{% endtab %}
{% endtabs %}

Let's take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.3, 5.9], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([2.1, 6.3], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.NORTH))

    # Undock
    navigator.undock()

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()
```

This example is very similar to [Navigate Through Poses](#navigate-through-poses). The difference is that we are using different poses as our waypoints, and that we use the `startFollowWaypoints` method to perform our navigation behaviour.


### Watch navigation progress in Rviz

You can visualise the navigation process in Rviz. If Rviz wasn't already launched, launch it by calling:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

{% tabs navigation %}
{% tab navigation galactic %}

<figure class="aligncenter">
    <img src="media/follow_waypoint_rviz.gif" alt="Follow Waypoints" style="width: 100%"/>
    <figcaption>Follow a set of Waypoints</figcaption>
</figure>

{% endtab %}
{% tab navigation humble %}

<figure class="aligncenter">
    <video style="width: 100%" controls autoplay muted>
    <source src="media/follow_waypoint_rviz_humble.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
    <figcaption>Follow a set of Waypoints</figcaption>
</figure>

{% endtab %}
{% endtabs %}


## Create Path

This example demonstrates how to create a navigation path in Rviz during runtime. It uses the [2D Pose Estimate](./navigation.md/#2d-pose-estimate) tool to pass the TurtleBot 4 Navigator a set of poses. Then we use the [Follow Waypoints](#follow-waypoints) behaviour to follow those poses. This example was run on a physical TurtleBot 4.

To run this example, start [Navigation](./navigation.md#launching-navigation) on your PC or on the Raspberry Pi using a map of your environment.

Once the navigation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials create_path
```

On your PC you will need to start Rviz:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

### Code breakdown

{% tabs navigation %}
{% tab navigation galactic %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/create_path.py).

{% endtab %}
{% tab navigation humble %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/humble/turtlebot4_python_tutorials/turtlebot4_python_tutorials/create_path.py).

{% endtab %}
{% endtabs %}

Let's take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Set goal poses
    goal_pose = navigator.createPath()

    if len(goal_pose) == 0:
        navigator.error('No poses were given, exiting.')
        exit(0)

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.clearAllCostmaps()
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startFollowWaypoints(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()
```

This example begins the same as the others by initialising the TurtleBot 4 Navigator.

#### Create your path

After initialisation, the user is prompted to create their path by using the [2D Pose Estimate](./navigation.md/#2d-pose-estimate) tool. You must set at least one pose. Once all of the poses have been set, the robot will begin navigating.

```py
goal_pose = navigator.createPath()

if len(goal_pose) == 0:
    navigator.error('No poses were given, exiting.')
    exit(0)
```

#### Set initial pose and clear costmaps

Next we set the initial pose and clear all costmaps. We clear costmaps because the 2D Pose Estimate tool is subscribed to by the Nav2 stack, and every time we use it Nav2 assumes that the robot is in that position, when it is not. Clearing the costmaps will get rid of any false costmaps that may have spawned when creating the path.

```py
if not navigator.getDockedStatus():
    navigator.info('Docking before initialising pose')
    navigator.dock()

initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
navigator.clearAllCostmaps()
navigator.setInitialPose(initial_pose)

navigator.waitUntilNav2Active()
```

We also wait for Nav2 to be active before continuing.

#### Follow the path

Now we can undock and follow the created path. In this example we use the [Follow Waypoints](#follow-waypoints) behaviour, but this can easily be replaced with [Navigate Through Poses](#navigate-through-poses).

```py
navigator.undock()

navigator.startFollowWaypoints(goal_pose)

navigator.dock()
```

We finish the example by docking the robot. This assumes that the last pose in the created path is near the dock. If it is not, you can remove this action.


### Creating a path with Rviz

Running this example will look something like this:

{% tabs navigation %}
{% tab navigation galactic %}

<figure class="aligncenter">
    <img src="media/create_path_rviz.gif" alt="Create Path" style="width: 100%"/>
    <figcaption>Creating a path and following it</figcaption>
</figure>

{% endtab %}
{% tab navigation humble %}

<figure class="aligncenter">
    <video style="width: 100%" controls autoplay muted>
    <source src="media/create_path_rviz_humble.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
    <figcaption>Creating a path and following it</figcaption>
</figure>

{% endtab %}
{% endtabs %}

```note
As the path is created, you will see the robot being placed at the position you click on. This is normal and gets cleared up when the initial pose is set by the TurtleBot 4 Navigator.
```

## Mail Delivery

{% tabs navigation %}
{% tab navigation galactic %}

This tutorial is only available in Humble

{% endtab %}
{% tab navigation humble %}

This example demonstrates how to create an interactive delivery route. The user is able to send the robot to different pre-defined locations on demand using the terminal interface. It then uses the [Navigate to Pose](#navigate-to-pose) behaviour to navigate to the pose.

To run this example, start the Ignition simulation:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials mail_delivery
```

{% endtab %}
{% endtabs %}

### Code breakdown

{% tabs navigation %}
{% tab navigation galactic %}

This tutorial is only available in Humble

{% endtab %}
{% tab navigation humble %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/humble/turtlebot4_python_tutorials/turtlebot4_python_tutorials/mail_delivery.py).

Let's take a look at the main function.

```py
def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Prepare goal pose options
    goal_options = [
        {'name': 'Home',
         'pose': navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST)},

        {'name': 'Position 1',
         'pose': navigator.getPoseStamped([10.0, 6.0], TurtleBot4Directions.EAST)},

        {'name': 'Position 2',
         'pose': navigator.getPoseStamped([-9.0, 9.0], TurtleBot4Directions.NORTH)},

        {'name': 'Position 3',
         'pose': navigator.getPoseStamped([-12.0, 2.0], TurtleBot4Directions.NORTH_WEST)},

        {'name': 'Position 4',
         'pose': navigator.getPoseStamped([3.0, -7.0], TurtleBot4Directions.WEST)},

        {'name': 'Exit',
         'pose': None}
    ]

    navigator.info('Welcome to the mail delivery service.')

    while True:
        # Create a list of the goals for display
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Prompt the user for the goal location
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verify that the value input is a number
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verify that the user input is within a valid range
        if (selected_index < 0) or (selected_index >= len(goal_options)):
            navigator.error(f'Goal selection out of bounds: {selected_index}')

        # Check for exit
        elif goal_options[selected_index]['name'] == 'Exit':
            break

        else:
            # Navigate to requested position
            navigator.startToPose(goal_options[selected_index]['pose'])

    rclpy.shutdown()
```
This example starts the same as [navigate to pose](#navigate-to-pose). We initialise the node, make sure the robot is docked, and set the initial pose. Then we wait for Nav2 to become active.

#### Prepare Goal Poses

The next step is to create a list of `PoseStamped` messages which represent all possible poses that the robot can be sent to.

```py
goal_options = [
    {'name': 'Home',
        'pose': navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST)},

    {'name': 'Position 1',
        'pose': navigator.getPoseStamped([10.0, 6.0], TurtleBot4Directions.EAST)},

    {'name': 'Position 2',
        'pose': navigator.getPoseStamped([-9.0, 9.0], TurtleBot4Directions.NORTH)},

    {'name': 'Position 3',
        'pose': navigator.getPoseStamped([-12.0, 2.0], TurtleBot4Directions.NORTH_WEST)},

    {'name': 'Position 4',
        'pose': navigator.getPoseStamped([3.0, -7.0], TurtleBot4Directions.WEST)},

    {'name': 'Exit',
        'pose': None}
]
```

#### Select the Goal Pose

The remainder of the program is repeated in a loop. 

A list of goals is compiled and displayed on the terminal as a prompt and the program will wait for input from the user. 

```py
# Create a list of the goals for display
options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
for i in range(len(goal_options)):
    options_str += f'    {i}. {goal_options[i]["name"]}\n'

# Prompt the user for the goal location
raw_input = input(f'{options_str}Selection: ')
```
The terminal will display:

<figure class="aligncenter">
    <img src="media/mail_delivery_terminal.png" alt="Mail Delivery Terminal" style="width: 100%"/>
    <figcaption>Mail Delivery Terminal Prompt</figcaption>
</figure>

The user will then enter a number and press enter. 

#### User Input Validation

The user input is verified to be an integer which is within the allowable range. If it is not a number or if the number does not correspond to a valid option then the loop will restart, prompting the user for a new input.

```py
selected_index = 0

# Verify that the value input is a number
try:
    selected_index = int(raw_input)
except ValueError:
    navigator.error(f'Invalid goal selection: {raw_input}')
    continue

# Verify that the user input is within a valid range
if (selected_index < 0) or (selected_index >= len(goal_options)):
    navigator.error(f'Goal selection out of bounds: {selected_index}')
```

#### Check for exit

If the user selected to exit the program then the loop is terminated and the program will exit.

```py
# Check for exit
elif goal_options[selected_index]['name'] == 'Exit':
    break
```

### Watch Navigation Progress in RViz

<figure class="aligncenter">
    <video style="width: 100%" controls autoplay muted>
    <source src="media/mail_delivery_rviz.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
    <figcaption>Mail Delivery (3x speed)</figcaption>
</figure>


#### Navigate

Finally, navigate to the selected position. 

```py
else:
    # Navigate to requested position
    navigator.startToPose(goal_options[selected_index]['pose'])
```

The loop then repeats, prompting the user for the next goal position.

{% endtab %}
{% endtabs %}


## Patrol Loop

{% tabs navigation %}
{% tab navigation galactic %}

This tutorial is only available in Humble

{% endtab %}
{% tab navigation humble %}

This example demonstrates how to create an infinite patrol loop with auto-charging. The robot will continuously drive through a set of poses until the charge is low and then it will go charge. Once the robot is sufficiently charged, it will start driving through the set of poses again. It uses the [Navigate to Pose](#navigate-to-pose) behaviour to navigate to the pose.

To run this example, start the Ignition simulation:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials patrol_loop
```

{% endtab %}
{% endtabs %}

### Code breakdown

{% tabs navigation %}
{% tab navigation galactic %}

This tutorial is only available in Humble

{% endtab %}
{% tab navigation humble %}

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/humble/turtlebot4_python_tutorials/turtlebot4_python_tutorials/patrol_loop.py).

#### Battery Monitoring Node
Let's take a look at the battery monitoring node class.

```py
class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')

        self.lock = lock

        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()
```
This class serves to monitor the Create3® battery charge percentage and to make it available for decision making. 

##### Subscription

The class contains a subscriber that listens on the `battery_state` topic for the latest battery state published by the Create3® and calls the `battery_state_callback` function whenever new information has been published.

```py
# Subscribe to the /battery_state topic
self.battery_state_subscriber = self.create_subscription(
    BatteryState,
    'battery_state',
    self.battery_state_callback,
    qos_profile_sensor_data)
```

##### Battery State Callback

Whenever a `battery_state` message is received, the battery charge percentage is saved in a member variable. 

A Global Interpreter Lock (GIL) is used to ensure that the main function and battery monitoring functions do not access or modify that member variable at the same time. The process will queue until the lock is available. 

```py
# Callbacks
def battery_state_callback(self, batt_msg: BatteryState):
    with self.lock:
        self.battery_percent = batt_msg.percentage
```

##### Prepare Threading

In order to listen for new messages and run callbacks, a node must be actively spinning. To simultaneously spin the node and run the navigation code, multi-threading is used. Here the battery node is prepared to spin in a separate thread which is created in the main function (see below).

```py
def thread_function(self):
    executor = SingleThreadedExecutor()
    executor.add_node(self)
    executor.spin()

```

#### Main Function
Let's take a look at the main function.

```py
def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    navigator = TurtleBot4Navigator()
    battery_percent = None
    position_index = 0

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Prepare goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-5.0, 1.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-5.0, -23.0], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([9.0, -23.0], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([10.0, 2.0], TurtleBot4Directions.WEST))

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if (battery_percent is not None):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

            # Check battery charge level
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('Battery critically low. Charge or power down')
                break
            elif (battery_percent < BATTERY_LOW):
                # Go near the dock
                navigator.info('Docking for charge')
                navigator.startToPose(navigator.getPoseStamped([-1.0, 1.0],
                                      TurtleBot4Directions.EAST))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Robot failed to dock')
                    break

                # Wait until charged
                navigator.info('Charging...')
                battery_percent_prev = 0
                while (battery_percent < BATTERY_HIGH):
                    sleep(15)
                    battery_percent_prev = floor(battery_percent*100)/100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    # Print charge level every time it increases a percent
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

                # Undock
                navigator.undock()
                position_index = 0

            else:
                # Navigate to next position
                navigator.startToPose(goal_pose[position_index])

                position_index = position_index + 1
                if position_index >= len(goal_pose):
                    position_index = 0

    battery_monitor.destroy_node()
    rclpy.shutdown()
```
This example starts with many of the same steps as [navigate to pose](#navigate-to-pose). We initialise the node, make sure the robot is docked, and set the initial pose. Then we wait for Nav2 to become active. The additional steps are discussed below. 

##### Multi-threading

 A Global Interpreter Lock (GIL) is used to ensure that the main function and battery monitoring functions do not access or modify that member variable at the same time.

 The lock is created as well as the battery monitor node being created. The lock is passed into the battery monitor node to ensure that both processes are using the same lock. 


```py
lock = Lock()
battery_monitor = BatteryMonitor(lock)
```

A thread is created to run the battery monitor thread function that was created earlier. This thread is started. 

```py
thread = Thread(target=battery_monitor.thread_function, daemon=True)
thread.start()
```

##### Prepare Goal Pose Loop

The robot poses that make up the robot's patrol loop are assembled into a list.

```py
# Prepare goal poses
goal_pose = []
goal_pose.append(navigator.getPoseStamped([-5.0, 1.0], TurtleBot4Directions.EAST))
goal_pose.append(navigator.getPoseStamped([-5.0, -23.0], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([9.0, -23.0], TurtleBot4Directions.NORTH_WEST))
goal_pose.append(navigator.getPoseStamped([10.0, 2.0], TurtleBot4Directions.WEST))
```

##### Operating Loop
The remainder of the code continues until it is interrupted.

The battery percentage is updated when it is granted access by the lock.

```py
with lock:
    battery_percent = battery_monitor.battery_percent
```
Initially when the processes are not ready, the battery percent may be None. The program continues to check until the battery percent is not None. 

```py
if (battery_percent is not None):

```

Based on the battery charge level, the program chooses an action: 

If the battery is at critical levels, it breaks the loop, ending the program.

```py
navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

# Check battery charge level
if (battery_percent < BATTERY_CRITICAL):
    navigator.error('Battery critically low. Charge or power down')
    break
```

If the battery is low then it navigates near the charger, docks and then waits for full charge. While charging it continues to periodically use the lock to update the battery percentage. Once full, it undocks and prepares to start the patrol loop again from the first position.

```py
elif (battery_percent < BATTERY_LOW):
    # Go near the dock
    navigator.info('Docking for charge')
    navigator.startToPose(navigator.getPoseStamped([-1.0, 1.0],
                            TurtleBot4Directions.EAST))
    navigator.dock()

    if not navigator.getDockedStatus():
        navigator.error('Robot failed to dock')
        break

    # Wait until charged
    navigator.info('Charging...')
    battery_percent_prev = 0
    while (battery_percent < BATTERY_HIGH):
        sleep(15)
        battery_percent_prev = floor(battery_percent*100)/100
        with lock:
            battery_percent = battery_monitor.battery_percent

        # Print charge level every time it increases a percent
        if battery_percent > (battery_percent_prev + 0.01):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

    # Undock
    navigator.undock()
    position_index = 0
```

Finally, if the battery was not critical or low then the robot navigates to the next position in the loop. This position is tracked by the 'position_index' variable

```py
else:
    # Navigate to next position
    navigator.startToPose(goal_pose[position_index])

    position_index = position_index + 1
    if position_index >= len(goal_pose):
        position_index = 0
```

### Watch Navigation Progress in RViz

<figure class="aligncenter">
    <video style="width: 100%" controls autoplay muted>
    <source src="media/patrol_loop_rviz.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
    <figcaption>Patrol Loop (9x speed)</figcaption>
</figure>

{% endtab %}
{% endtabs %}