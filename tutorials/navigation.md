---
sort: 5
---

# Navigation

This tutorial will cover various methods of navigating with the TurtleBot 4 and Nav2.

## SLAM vs Localization

There are two localization methods we can use to figure out where the robot is on the map: SLAM or Localization. SLAM allows us to generate the map as we navigate, while localization requires that a map already exists.

### SLAM

SLAM is useful for generating a new map, or navigating in unknown or dynamic environments. It updates the map as it detects and changes, but cannot see areas of the environment that it has not discovered yet. 

### Localization

Localization uses an existing map along with live odometry and laserscan data to figure out the position of the robot on the given map. It does not update the map if any changes have been made to the environment, but we can still avoid new obstacles when navigating. Because the map doesn't change, we can get more repeatable navigation results.

For this tutorial, we will be using localization to navigate on a map [generated with SLAM](generate_map.md#generating-a-map).

## Nav2

The TurtleBot 4 uses the [Nav2](https://navigation.ros.org/) stack for navigation.

### Launching navigation

{% tabs navigation %}
{% tab navigation galactic %}

For this tutorial we can launch navigation with [Nav Bringup](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/launch/nav_bringup.launch.py).

On a physical TurtleBot 4, call:

```bash
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=office.yaml
```

Replace `office.yaml` with your own map.

If you are using the simulator, call:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav2:=true slam:=off localization:=true
```

This will launch the simulation in the default `depot` world and will use the existing `depot.yaml` file for the map. If you are using a different world you will need to create a map for it and pass that in as a launch argument.

For example:
```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav2:=true slam:=off localization:=true world:=classroom map:=classroom.yaml
```

{% endtab %}
{% tab navigation humble %}

For this tutorial we can launch navigation with the [turtlebot4_navigation](https://github.com/turtlebot/turtlebot4/tree/humble/turtlebot4_navigation) package.

Open a terminal and launch [localization](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/localization.launch.py):

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=office.yaml
```

Then, in another terminal, launch [nav2](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/launch/nav2.launch.py):

```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

{% endtab %}
{% endtabs %}

```note
An initial pose is required before navigation can begin.
```

### Interacting with Nav2

In a new terminal, launch Rviz so that you can view the map and interact with navigation:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

<figure class="aligncenter">
    <img src="media/office.png" alt="Office Map" style="width: 100%"/>
    <figcaption>Office Map shown in Rviz</figcaption>
</figure>

At the top of the Rviz window is the toolbar. You will notice that there are three navigation tools available to you.

<figure class="aligncenter">
    <img src="media/nav_tools.png" alt="Navigation tools" style="width: 60%"/>
    <figcaption>Navigation tools in Rviz</figcaption>
</figure>

#### 2D Pose Estimate

The 2D Pose Estimate tool is used in localization to set the approximate initial pose of the robot on the map. This is required for the Nav2 stack to know where to start localizing from. Click on the tool, and then click and drag the arrow on the map to approximate the position and orientation of the robot.

<figure class="aligncenter">
    <img src="media/pose_estimate.gif" alt="Pose Estimate" style="width: 100%"/>
    <figcaption>Setting the initial pose</figcaption>
</figure>

#### Publish Point

The Publish Point tool allows you to click on a point on the map, and have the coordinates of that point published to the `/clicked_point` topic.

Open a new terminal and call:

```bash
ros2 topic echo /clicked_point
```

Then, select the Publish Point tool and click on a point on the map. You should see the coordinates published in your terminal.

<figure class="aligncenter">
    <img src="media/clicked_point.gif" alt="Publish Point" style="width: 100%"/>
    <figcaption>Getting a point coordinate</figcaption>
</figure>


#### Nav2 Goal

The Nav2 Goal tool allows you to set a goal pose for the robot. The Nav2 stack will then plan a path to the goal pose and attempt to drive the robot there. Make sure to set the initial pose of the robot before you set a goal pose.

<figure class="aligncenter">
    <img src="media/nav_goal.gif" alt="Nav2 Goal" style="width: 100%"/>
    <figcaption>Driving the TurtleBot4 with a Nav2 Goal</figcaption>
</figure>

## TurtleBot 4 Navigator

The [TurtleBot 4 Navigator](https://github.com/turtlebot/turtlebot4/blob/galactic/turtlebot4_navigation/turtlebot4_navigation/turtlebot4_navigator.py) is a Python node that adds on to the [Nav2 Simple Commander](https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/robot_navigator.py). It includes TurtleBot 4 specific features such as docking and undocking, as well as easy to use methods for navigating.

```note
TurtleBot 4 Navigator requires at least version 1.0.11 of Nav2 Simple Commander
```

The following examples can be installed with `sudo apt install ros-$ROS_DISTRO-turtlebot4-tutorials` and are available at <https://github.com/turtlebot/turtlebot4_tutorials>. For each example, the robot starts on a dock at the origin of the map. 

### Navigate to Pose

This example demonstrates the same behaviour as [Nav2 Goal](#nav2-goal). The Nav2 stack is given a pose on the map with which it calculates a path. The robot then attempts to drive along the path. This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

To run this example, start the Ignition simulation:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav:=true slam:=off localization:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials nav_to_pose
```

#### Code breakdown

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/nav_to_pose.py).


Lets take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
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

##### Initialise the node

We start by initialising `rclpy` and creating the `TurtleBot4Navigator` object. This will initialise any ROS 2 publishers, subscribers and action clients that we need.

```py
rclpy.init()

navigator = TurtleBot4Navigator()
```

##### Dock the robot

Next, we check if the robot is docked. If it is not, we send an action goal to dock the robot. By docking the robot we guarantee that it is at the [0.0, 0.0] coordinates on the map.

```py
if not navigator.getDockedStatus():
    navigator.info('Docking before intialising pose')
    navigator.dock()
```

##### Set the initial pose

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

##### Wait for Nav2

Once the initial position has been set, the Nav2 stack will place the robot at that position on the map and begin localizing. We want to wait for Nav2 to be ready before we start sending navigation goals.

```py
navigator.waitUntilNav2Active()
```

```note
This call will block until Nav2 is ready. Make sure you have launched nav bringup in a separate terminal.
```

##### Set the goal pose

Now we can create a [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/galactic/geometry_msgs/msg/PoseStamped.msg) message. The `getPoseStamped` method makes it easy for us. All we have to do is pass in a list describing the x and y position that we want to drive to on the map, and the direction that we want the robot to be facing when it reaches that point.

```py
goal_pose = navigator.getPoseStamped([13.0, 5.0], TurtleBot4Directions.EAST)
```

##### Undock the robot and go to the goal pose

We are ready to drive to the goal pose. We start by undocking the robot so that it does not attempt to drive through the dock, and then send the goal pose. As the robot drives to the goal pose, we will be receiving feedback from the action. This feedback includes the estimated time of arrival.

```py
navigator.undock()

navigator.startToPose(goal_pose)
```

Once the robot has reached the goal, we call `rclpy.shutdown()` to gracefully destroy the rclpy context.

#### Watch navigation progress in Rviz

You can visualise the navigation process in Rviz by calling:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

<figure class="aligncenter">
    <img src="media/nav_to_pose_rviz.gif" alt="Nav to pose" style="width: 100%"/>
    <figcaption>Navigate to a pose</figcaption>
</figure>

### Navigate Through Poses

This example demonstrates the [Navigate Through Poses](https://navigation.ros.org/behavior_trees/trees/nav_through_poses_recovery.html) behaviour tree. The Nav2 stack is given a set of poses on the map and creates a path that goes through each pose in order until the last pose is reached. The robot then attempts to drive along the path. This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

To run this example, start the Ignition simulation:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav:=true slam:=off localization:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials nav_through_poses
```

#### Code breakdown

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/nav_through_poses.py).


Lets take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
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

This example starts the same as [navigate to pose](#navigate-to-pose). We initialse the node, make sure the robot is docked, and set the initial pose. Then we wait for Nav2 to become active.

##### Set goal poses

The next step is to create a list of `PoseStamped` messages which represent the poses that the robot needs to drive through.

```py
goal_pose = []
goal_pose.append(navigator.getPoseStamped([0.0, -1.0], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([1.7, -1.0], TurtleBot4Directions.EAST))
goal_pose.append(navigator.getPoseStamped([1.6, -3.5], TurtleBot4Directions.NORTH))
goal_pose.append(navigator.getPoseStamped([6.75, -3.46], TurtleBot4Directions.NORTH_WEST))
goal_pose.append(navigator.getPoseStamped([7.4, -1.0], TurtleBot4Directions.SOUTH))
goal_pose.append(navigator.getPoseStamped([-1.0, -1.0], TurtleBot4Directions.WEST))
```

##### Navigate through the poses

Now we can undock the robot and begin navigating through each point. Once the robot has reached the final pose, it will then return to the dock.

```py
navigator.undock()

navigator.startThroughPoses(goal_pose)

navigator.dock()
```

#### Watch navigation progress in Rviz

You can visualise the navigation process in Rviz by calling:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

<figure class="aligncenter">
    <img src="media/nav_through_pose_rviz.gif" alt="Nav through pose" style="width: 100%"/>
    <figcaption>Navigate through a set of poses</figcaption>
</figure>

### Follow Waypoints

This example demonstrates how to follow waypoints. The Nav2 stack is given a set of waypoints on the map and creates a path that goes through each waypoint in order until the last waypoint is reached. The robot then attempts to drive along the path. The difference between this example and Navigating Through Poses is that when following waypoints the robot will plan to reach each waypoint individually, rather than planning to reach the last pose by driving through the other poses.  This example is demonstrated in the `depot` world of the TurtleBot 4 simulation.

To run this example, start the Ignition simulation:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav:=true slam:=off localization:=true
```

Once the simulation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials follow_waypoints
```

#### Code breakdown

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/follow_waypoints.py).


Lets take a look at the main function.

```py
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
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


#### Watch navigation progress in Rviz

You can visualise the navigation process in Rviz by calling:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

<figure class="aligncenter">
    <img src="media/follow_waypoint_rviz.gif" alt="Follow Waypoints" style="width: 100%"/>
    <figcaption>Follow a set of Waypoints</figcaption>
</figure>

### Create Path

This example demonstrates how to create a navigation path in Rviz during runtime. It uses the [2D Pose Estimate](#2d-pose-estimate) tool to pass the TurtleBot 4 Navigator a set of poses. Then we use the [Follow Waypoints](#follow-waypoints) behaviour to follow those poses. This example was run on a physical TurtleBot 4.

To run this example, start [Navigation](./navigation.md#launching-navigation) on your PC or on the Raspberry Pi using a map of your environment.

Once the navigation has started, open another terminal and run:

```bash
ros2 run turtlebot4_python_tutorials create_path
```

On your PC you will need to start Rviz:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### Code breakdown

The source code for this example is available [here](https://github.com/turtlebot/turtlebot4_tutorials/blob/galactic/turtlebot4_python_tutorials/turtlebot4_python_tutorials/create_path.py).


Lets take a look at the main function.

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
        navigator.info('Docking before intialising pose')
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

##### Create your path

After initialisation, the user is prompted to create their path by using the [2D Pose Estimate](#2d-pose-estimate) tool. You must set at least one pose. Once all of the poses have been set, the user can press CTRL + C to stop creating the path and begin navigating.

```py
goal_pose = navigator.createPath()

if len(goal_pose) == 0:
    navigator.error('No poses were given, exiting.')
    exit(0)
```

##### Set initial pose and clear costmaps

Next we set the initial pose and clear all costmaps. We clear costmaps because the 2D Pose Estimate tool is subscribed to by the Nav2 stack, and every time we use it Nav2 assumes that the robot is in that position, when it is not. Clearing the costmaps will get rid of any false costmaps that may have spawned when creating the path.

```py
if not navigator.getDockedStatus():
    navigator.info('Docking before intialising pose')
    navigator.dock()

initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
navigator.clearAllCostmaps()
navigator.setInitialPose(initial_pose)

navigator.waitUntilNav2Active()
```

We also wait for Nav2 to be active before continuing.

##### Follow the path

Now we can undock and follow the created path. In this example we use the [Follow Waypoints](#follow-waypoints) behaviour, but this can easily be replaced with [Navigate Through Poses](#navigate-through-poses).

```py
navigator.undock()

navigator.startFollowWaypoints(goal_pose)

navigator.dock()
```

We finish the example by docking the robot. This assumes that the last pose in the created path is near the dock. If it is not, you can remove this action.


#### Creating a path with Rviz

Running this example will look something like this:

<figure class="aligncenter">
    <img src="media/create_path_rviz.gif" alt="Create Path" style="width: 100%"/>
    <figcaption>Creating a path and following it</figcaption>
</figure>

```note
As the path is created, you will see the robot being placed at the position you click on. This is normal and gets cleared up when the initial pose is set by the TurtleBot 4 Navigator.
```