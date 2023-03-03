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

**SLAM estimate remains good even after colliding with an obstacle, messing up odometry:**
This result was achieved by first runnning the slam.launch.xml file, following by 
```
ros2 service call circle/control nuturtle_control/srv/Control "velocity: 0.08
radius: 0.5
"
```
![good_result](https://user-images.githubusercontent.com/45540813/222725895-fcf66a30-96df-4d8f-ae8d-5e3d57093ddb.png)


**SLAM pose estimate vs. time for robot driving in a circle:**
![circle_plot](https://user-images.githubusercontent.com/45540813/222717511-c264f501-a403-43ce-b61c-3c0515437540.png)

### Issues
As of right now there is a bug somewhere which is causing spikes in the
state estimate from the extended Kalman filter. With certain starting conditions,
driving the robot in a circle using the circle node will give worse results
and the pose estimate will jump around. This issue can be clearly seen in
the below plot. Furthermore, these two test were run using the `circle` node
and I have found these errors to be more pronounced when using
`teleop_twist_keyboard`.

![noise_spikes](https://user-images.githubusercontent.com/45540813/222746914-8f2250c0-b8b0-44df-8cea-d066968dd2ad.png)

