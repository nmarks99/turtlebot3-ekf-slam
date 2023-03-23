# EKF SLAM From Scratch
In this project, I implmement an extended Kalman filter (EKF) simultaneous localization
and mapping (SLAM) algorithm from scratch for the Turtlebot3 using C++ and ROS2.
This includes a simulator so the algorithm can be tested in simulation before
being tested on the real robot. A complete list of packages in this repository
is provided in the next section

# Package List
This repository consists of several ROS packages
- nusim - A turtlebot simulation program using RVIZ
- nuturtle_description - Defines the turtlebot's physical properties
- nurtle_control - Recieves body twist commands and converts them to commands to move the robot
- turtlelib
  - rigid2D: A 2D rigid body transformations library
  - diff_drive: A differential drive robot library
  - kalman: Extended Kalman Filter library
- nuslam - Extended Kalman Filter SLAM for estimating the robot's pose and map based on sensor data
  - circle_fitting: Groups LIDAR data points into clusters, fits a circle to them, and classifies them

For a more detailed description of these packages, see the README files in each respective package.
