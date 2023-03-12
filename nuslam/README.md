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
![no_noise_run](https://user-images.githubusercontent.com/45540813/224562029-d6b0110a-d232-4a40-b849-6cfca3bfb454.png)

![no_noise_plot](https://user-images.githubusercontent.com/45540813/224562073-2e211295-4909-4e4a-a41e-984da214eed2.png)



### With sensor noise and slipping
- Insert screenshot and plot here 
