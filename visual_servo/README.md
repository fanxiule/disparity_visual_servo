# Disparity-based Visual Servoing
This package contains files for the control part of the visual servoing scheme.

## Using the Package
After moving the robot to its desried pose, run the following command to record the pose as well as the reference disparity and occlusion map
```
roslaunch visual_servo record_ref.launch
```
The images will be saved in the `target/` directory. Some sample reference images are provided in this folder already.

After recording the target images, run the following command to start the controller
```
$ roslaunch visual_servo move_robot.launch
```

Or run the command below to start the controller AND record a rosbag with the robot's `/odom` and `cmd_vel` topics
```
$ roslaunch visual_servo move_robot_record.launch
```
The rosbag file is saved in this package with file name started with `robot_state`.

After recording the rosbag file, you can convert it to a more readable format by running
```
$ roslaunch visual_servo process_rosbag.launch
```
Remember to update the `bag_file` argument in the launch file to the correct rosbag file name. The converted files will be saved in the `robot_state` folder.