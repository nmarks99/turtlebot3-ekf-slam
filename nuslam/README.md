# NUSLAM
This ROS2 package does Extended Kalman Filter SLAM and publishes
the associated pose and map estimations for the robot and landmarks.

## Configuration
The `config/slam_params.yaml` file can be used to configure
various parameters for the node. When running in simulation, the simulator
can be configured through the `config/nusim_slam_params.yaml` file.

## Launchfile
To launch the SLAM node and other nodes needed to run it in the simulator,
with visualizations in RVIZ, run the following command:

```
ros2 launch nuslam slam.launch.xml use_rviz:=true
```

## Results

### SLAM estimate remains good even after colliding with an obstacle, messing up odometry:

**No noise:**
*insert video here*

*insert plot here*

**With sensor noise and slipping:**
*insert video here*

*insert plot here*
