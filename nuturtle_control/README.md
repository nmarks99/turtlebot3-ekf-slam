# nuturtle_control
This package subscribes to geoemetry_msg/Twist messages from
the /cmd_vel topic and computes the wheel speeds and commands needed
to achieve the desired twist, then published the wheel commands
on the /wheel_cmd topic.

## Launchfiles

### `start_robot.launch.xml`
Starts all the nodes required for sending Twist commands to
the robot, visualizing its motion in RVIZ, and computing odometry.