# ROS 2 - publishing data

![](_resources/lab6/ros-humble-hawksbill-featured.jpg)

## Setting up the environment

Before you start working with ROS, call the command in each newly opened terminal:

- after the first compilation (there are `build`, `install`, `log` directories):

```bash
source install/setup.bash
```

- before the first compilation:

```bash
source /opt/ros/humble/setup.bash
```

## Introduction

In this manual, we will publish data using *topics*. We will use different robots: turtlesim and simulated robotic arm.

Information about topics in ROS and `turtlesim` can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

## Publishing in the context of `turtlesim`

Run the simulation:

```bash
ros2 run turtlesim turtlesim_node
```

Test the simulation interactivity by running the `turtle_teleop_key` node in a separate terminal:

```bash
ros2 run turtlesim turtle_teleop_key
```

View the running nodes and the connections between them using the `rqt_graph` tool.

We can control our RoboTurtle using *topics*. Use the appropriate command to get a list of them

```bash
ros2 topic list
```

and display the current pose of the RoboTurtle:

```bash
ros2 topic echo /turtle1/pose
```

Use `Ctrl+c` to terminate the data subscription.

Wanting to publish data to the `/turtle1/cmd_vel` topic and move the RoboTurtle, we should first check the topic type:

```bash
ros2 topic type /turtle1/cmd_vel
```

The command returns the type `geometry_msgs/msg/Twist`. We can check the type with the following command:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

The `Twist` structure contains two 3D vectors. The first vector defines the linear velocity and the second defines the angular velocity. Our robotic turtle, unlike multi-rotor drones, is a non-holonomic system. This means that the robot can move forward/backward (`x` axis) and rotate around the `z` axis. To move the robot, you need to publish data to the `/turtle1/cmd_vel` theme by setting the appropriate values for linear and angular velocity:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

🐢🐢🐢

**You don't need to remember the type and structure of the message. Use the `Tab` button and the Linux terminal will help you with the correct names.**

🐢🐢🐢

Change the parameter values so that the robot turtle starts moving. Test different variations.

## Manipulating the robotic arm

Let's start by installing the driver for Universal Robots:

```bash
apt install ros-${ROS_DISTRO}-ur-robot-driver
```

After installation, we can start the `UR3` robot simulator. This step internally uses Docker software (in case of an error, make sure you have completed [docker configuration](https://docs.docker.com/engine/install/linux-postinstall/)):

```bash
ros2 run ur_robot_driver start_ursim.sh -m ur3
```

After starting the simulator, we can run the robot driver in a new terminal window:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 launch_rviz:=true
```

🐢🐢🐢

**If the `rviz` program shows a sad, curled up in a ball and textureless robotic arm try aborting the above script and running it again. After restarting, wait a few seconds. If restarting several times doesn't help, open the robot panel in your browser (the link displays the command that starts the simulator) and try to move the robot with the buttons in the panel. Restart the controller with the visualization (potentially several times).**

🐢🐢🐢

The browser-launched robot panel (PolyScope) is a graphical user interface (GUI) for controlling the robot arm, executing robot programs and easily creating new ones. A properly run simulation should allow you to control the robot in PolyScope and observe its movements in RViz.

## Tasks

1. Read the frequency of the `UR3` robot control loop. To do this, display the frequency for the subject `/joint_states`.
2. Move the arm in the `UR3` panel. Verify that `rviz` correctly renders the robot's pose.
3. In the terminal, check what topics and what types the `UR3` controller has created.
4. Set the arm to the `Home` position using the `UR3` panel.
5. Try to run the sample trajectory using the command: `ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py`. Does the robot move?
6. Use the `UR3` panel to create a new, empty program. Add the `External Control` command to it (from `Structure` -> `URCaps`). Run the program and the example trajectory from the previous task. Does the robot move in the panel and `rviz`?
7. Test the above on another `UR` robot, for example `ur5e`.
