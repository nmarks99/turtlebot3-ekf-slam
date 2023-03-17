# NUSLAM
This ROS2 package does Extended Kalman Filter SLAM and publishes
the associated pose and map estimations for the robot and landmarks.

## Configuration
The `config/slam_params.yaml` file can be used to configure
various parameters for the node. When running in simulation, the simulator
for SLAM can be configured through the `config/nusim_slam_params.yaml` file.

## SLAM with Known Data Association
To launch the SLAM node and other nodes needed to run it in the simulator,
with visualizations in RVIZ, run the following command:

```
ros2 launch nuslam slam.launch.xml use_rviz:=true
```

### No noise or slipping
![no_noise_run](https://user-images.githubusercontent.com/45540813/224562029-d6b0110a-d232-4a40-b849-6cfca3bfb454.png)

![no_noise_plot](https://user-images.githubusercontent.com/45540813/224562073-2e211295-4909-4e4a-a41e-984da214eed2.png)


### With sensor noise and slipping
![slam_good_with_noise](https://user-images.githubusercontent.com/45540813/225810142-71e0eefd-bd22-4f12-90d9-a50f69bef441.png)



## Landmark Detection
The landmarks node is responsible for subscribing to LIDAR data (either real
or simulated) and using it to determine where the landmarks are. This node uses
the circle fitting library also located in the `nuslam` package. The circle
fitting code clusters points, fits a circle to them, and classifies if the resulting
fit is truly a circle. If it is, the center of the fitted circle is used as the
landmark location and published on the /detected_landmarks topic which is subscribed
to by the slam node. Finally, the slam node passes this detected landmark location to
the extended Kalman filter algorithm which handles the data association.

To run a demonstration of the circle fitting code and to visualize the clusters
in RVIZ, run the following command:

```
ros2 launch nuslam landmarks_detect.launch.xml
```

The image below shows the result of the running the below launchfile.
The dark green spheres represent the centroid of the clusters detected
by the algorithm. A custom message which defines an array of geometry_msg/Point
messages of the centers of the detected circles is pubished on /detected_landmarks.
for the case as shown in the image, `ros2 topic echo /detected_landmarks`
produces the following:
```
points:
- x: 0.2988810133995391
  y: 0.49874838011065054
  z: 0.0
- x: -0.49959055108359973
  y: -0.24956688717167066
  z: 0.0
```
In the image the markers near the green spheres are located at (0.3,0.5)
and (-0.5,-0.25). Since the simulated LIDAR sensor variance in this example is
set to 0.001, the noise is quite low and we can see the predicted centers align
very closely with the true centers.
![landmark_detection](https://user-images.githubusercontent.com/45540813/225810177-f3f4e44b-e4df-422a-8238-0337ad411027.png)


## Data Association
After the landmarks have been detected through clustering, circle fitting, and
circle classification, each measurment must be associated with previously seen landmarks
or added as a new landmark. Unfortunately, I was not able to get this aspect of the
project working in the end.

