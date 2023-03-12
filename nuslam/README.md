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

### No noise or slipping
- insert video here

![no_noise_collision_plot](https://user-images.githubusercontent.com/45540813/224561432-b61cfc50-321b-4f51-aa18-213fe21d4490.png)

### With sensor noise and slipping
- Insert video and plot here
