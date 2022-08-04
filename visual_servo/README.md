# Disparity-based Visual Servoing
This package contains files for the control part of the visual servoing scheme.

## Using the Package
Before running any code, remember to start up the robot first by
```
$ roslaunch turtlebot_bringup minimal.launch
```

After moving the robot to its desried pose, run the following command to record the pose as well as the reference disparity and occlusion map
```
roslaunch visual_servo record_ref.launch
```
The images will be saved in the `target/` directory. Some sample images are provided in this folder.

After recording the target images, run the following command to start the controller for the positioning task
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

To collect target images for the navigation task, first start the camera and allow teleoperation by running
```
$ roslaunch crd_fusion_perception refine_rs_disp.launch
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```
When moving the robot to one of the target poses, run
```
$ roslaunch visual_servo save_sequence_manual.launch
```
This will save the current images as one of the target set in the `sequence/` folder. After the entire sequence is recorded, run the navigation experiment by
```
$ roslaunch visual_servo move_robot_sequence.launch
```
Or 
```
$ roslaunch visual_servo move_robot_sequence_record.launch
```
to record a rosbag file with its name started with `sequence`, which can be processed by
```
$ roslaunch visual_servo process_sequence.launch
```
 