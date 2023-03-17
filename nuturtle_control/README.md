# nuturtle_control
This package subscribes to geoemetry_msg/Twist messages from
the /cmd_vel topic and computes the wheel speeds and commands needed
to achieve the desired twist, then published the wheel commands
on the /wheel_cmd topic.

## Launchfiles

### `start_robot.launch.xml`
Starts all the nodes required for sending Twist commands to
the robot, visualizing its motion in RVIZ, and computing odometry.

## Demonstation of turtlebot driving in a circle
The video below shows a demonstration of the turtlebot driving in a a circle, and
using several services I wrote, reversing direction and stopping. At the end I use 
the `teleop_twist_keyboard` program to attempt to move the robot back to where it
started.

<video src=https://user-images.githubusercontent.com/45540813/217746813-e4856ca9-38ba-4b2a-a826-e1defa4409de.mp4/>
