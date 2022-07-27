# Visual Servoing using Accurate Disparity Images

This repository includes the required code to run an occlusion-aware disparity-based direct visual servoing algorithm for a mobile robot in a simulation environment using ROS, PyTorch, and Gazebo.

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

Organize the data and train the network by following instructions on the [CRD-Fusion](https://github.com/fanxiule/CRD_Fusion) repository.

Some pretrained weights are already saved in the `crd_fusion_perception` package.

## RUNNING THE CONTROLLER

First, launch the simulator in Gazebo by running
```
$ roslaunch turtlebot3_d435 turtlebot3_d435_ref_disp.launch
```
This command starts the Gazebo environment with the Turtlebot3 Burger and an Intel D435 camera attached to the robot. Additionally, 
the CRD-Fusion perception pipeline utilizes the stereo camera to compute accurate disparity maps. 

To start the disparity-based controller, run
```
$ roslaunch visual_servo move_robot.launch
```
This command will run the controller by using the target images saved in the `visual_servo` package as the reference signals.