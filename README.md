# Visual Servoing using Accurate Depth Maps

## DEPENDENCIES

Install ROS, Turtlebot3 packages, and Turtlebot3 siumulation packages according this [manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).

## RECORD TRAINING DATASET

Run the following commands
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch depth_visual_servo_common collect_dataset.launch
```
A Gazebo environment will start after running the above command. Meanwhile, a rosbag containing the stereo IR images, ground truth depth, and raw disparity is saved in the `depth_visual_servo_common` package. Use keyboard to teleoperate the robot. Then run the following command to convert the bag file
```
$ roslaunch depth_visual_servo_common convert_dataset.launch
```
Remember to change the `bag_file` argument in the launch file. The data in the bag file will be saved as images in the `depth_visual_servo_common/realsense`.