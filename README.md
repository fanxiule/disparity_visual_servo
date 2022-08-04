# Visual Servoing using Accurate Disparity Images

This repository includes the required code to run an occlusion-aware disparity-based direct visual servoing algorithm on a physical Turtlebot2 using ROS and PyTorch.

## DEPENDENCIES

Install ROS, PyTorch, and the Turtlebot2 packages.

You can refer to `pip_requirements.txt` and `pip3_requirements.txt` to see what we have in our `pip` and `pip3` environments.

Before running any code, connect the camera (D415) and robot to the host laptop. Then turn on the robot

## RECORD TRAINING DATASET

Run the following commands
```
$ roslaunch crd_fusion_perception rs_camera.launch
$ roslaunch depth_visual_servo_common collect_dataset.launch
```
You should be able to teleoperate the Turtlebot2. When the robot is running, the camera will process the input images. Meanwhile, a rosbag containing the stereo IR images, ground truth depth, and raw disparity is saved in the `depth_visual_servo_common` package. Use keyboard to teleoperate the robot. Then run the following command to convert the bag file
```
$ roslaunch depth_visual_servo_common convert_dataset.launch
```
Remember to change the `bag_file` argument in the launch file. The data in the bag file will be saved as images in the `depth_visual_servo_common/realsense`.

Organize the data and train the network by following instructions on the [CRD-Fusion](https://github.com/fanxiule/CRD_Fusion) repository.

Some pretrained weights are already saved in the `crd_fusion_perception` package.

## RUNNING THE CONTROLLER

First, launch the camera and the CRD-Fusion pipeline by running
```
$ roslaunch crd_fusion_perception refine_rs_disp.launch
```
Then, start the robot with
```
$ roslaunch turtlebot_bringup minimal.launch
```
There are two types of experiments you can run. Both of them use some pre-captured target images. You can run the positioning experiment by
```
$ roslaunch visual_servo move_robot.launch
```
Or run the navigation experiment  by
```
$ roslaunch visual_servo move_robot_sequence.launch
```