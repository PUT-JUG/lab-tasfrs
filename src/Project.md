# Project

## Description

The project should be done using ROS2 and include self-created nodes (or at least a single node). The use of external packages is also possible (e.g. `usb_cam`) and recommended. The goal of the project is to create an interface for controlling the robot. The process should look as follows:

1. Launch a camera driver (can be a laptop camera or an external camera).
2. Start your node(s) to control the robot.
3. Launch the robot driver.
4. Demonstrate how the interface works.

The node created should allow:

* in the simpler version or for those without access to a camera: click on a point in the window, depending on whether the point pressed is above the center of the screen or below, the robot moves forward or backward.
* in the default version: the ArUco marker is detected on the screen, depending on whether the center of the marker is above the center of the erakno or below, the robot moves forward or backward

## Example resources

 Example simulated robots that are also stationed in the lab can be used for the project:

* [UR 5](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
* [TurtleBot](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

 Depending on the chosen robot, the control method changes. For the mobile robot, we publish commands on the `/cmd_vel` topic, while for the UR 5 robot we publish commands on the `/scaled_joint_trajectory_controller/joint_trajectory` topic. For the manipulator, you can rotate the robot in the first axis (at the base) by a fixed angle, and for the mobile robot, move forward and backward. A version where the bottom position means the robot stops (motionless) is also acceptable.

 The simplest version involves creating a project using a robotic turtle [turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#).

## Form of rendering projects

## Example demonstration (simplified version for UR 5 robot)

![](_resources/project/demo.gif)
