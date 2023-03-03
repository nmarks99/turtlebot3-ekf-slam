# ME495 Sensing, Navigation and Machine Learning For Robotics
* Nick Marks
* Winter 2022
* Also worked with: Kevin, Ishaan

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

For a more detailed description of these packages, see the README files in each respective package.


## Demonstation of turtlebot driving in a circle
The video below shows a demonstration of the turtlebot driving in a a circle, and
using several services I wrote, reversing direction and stopping. At the end I use 
the `teleop_twist_keyboard` program to attempt to move the robot back to where it
started. Unfortunately, at the time of recording this video, my odometry was not 
working correctly so I do not have a value for how far the turtlebot is from its starting
configuration

<video src=https://user-images.githubusercontent.com/45540813/217746813-e4856ca9-38ba-4b2a-a826-e1defa4409de.mp4/>
