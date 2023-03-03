# NUSLAM
This ROS2 package does Extended Kalman Filter SLAM and publishes
the associated pose and map estimations for the robot and landmarks.
For now this is only setup to run in simulation but will later be
able to be used with the real Turtlebot3.

## Launchfile
To launch the SLAM node and other nodes needed to run it in the simulator,
with visualizations in RVIZ, run the following command:

```
ros2 launch nuslam slam.launch.xml use_rviz:=true
```

## Results
**Results go here**
