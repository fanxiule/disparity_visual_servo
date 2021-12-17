# Turtlebot3 with Intel RealSense D435 Camera
This package contains files necessary to run a Turtlebot3 in a simulation Gazebo environment with an Intel RealSense D435 Camera.

## Using the Package
Run the following command to launch the robot in a Gazebo environment with the camera
```
roslaunch turtlebot3_d435 turtlebot3_d435.launch
```

If you want to run the task above AND use a stereo matching algorithm provided by OpenCV to calculate disparity, use
```
roslaunch turtlebot3_d435 turtlebot3_d435_raw_disparity.launch
```