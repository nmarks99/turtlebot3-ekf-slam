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

**SLAM estimate remains good even after colliding with a wall, messing up odometry:**
![good_result](https://user-images.githubusercontent.com/45540813/222720971-91fd82dd-681c-4a19-a628-ef83d51bf8c0.png)

**SLAM pose estimate vs. time for robot driving in a circle:**
![circle_plot](https://user-images.githubusercontent.com/45540813/222717511-c264f501-a403-43ce-b61c-3c0515437540.png)


https://user-images.githubusercontent.com/45540813/222717700-ebc3ccf3-bc04-4a50-8ba4-08aec8aa493e.mp4

