# Occlusion-Aware Disparity-based Direct Visual Servoing of Mobile Robots

This repository includes the required code to run an occlusion-aware disparity-based direct visual servoing algorithm for a mobile robot in a simulation environment using ROS, PyTorch, and Gazebo.

This method was published in our [book chapter](https://www.taylorfrancis.com/chapters/edit/10.1201/9781003343783-10/occlusion-aware-disparity-based-direct-visual-servoing-mobile-robots-xiule-fan-baris-fidan-soo-jeon).
> Occlusion-Aware Disparity-based Direct Visual Servoing of Mobile Robots
> 
> by Xiule Fan, Baris Fidan, Soo Jeon
> 
> Measurements and Instrumentation for Machine Vision, 2024

## Dependencies

Install ROS, PyTorch, Turtlebot3 packages, and Turtlebot3 siumulation packages according this [manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). The method has been tested in Ubuntu 18.04, ROS Melodic, and PyTorch 1.9.

## Record Training Dataset

First, we need to collect a dataset in the Gazebo environment to fine tune [CRD-Fusion](https://github.com/fanxiule/CRD_Fusion) for disparity estimation. 

Run the following commands to collect this dataset
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch depth_visual_servo_common collect_dataset.launch
```
A Gazebo environment will start after running the above command. Meanwhile, a rosbag containing the stereo IR images, ground truth depth, and raw disparity is saved in the `depth_visual_servo_common` package. 

Use keyboard to teleoperate the robot. Then run the following command to convert the bag file into image files
```
$ roslaunch depth_visual_servo_common convert_dataset.launch
```
Remember to change the `bag_file` argument in the launch file. The data in the bag file will be saved as images in the `depth_visual_servo_common/realsense`.

Organize the data and train the network by following instructions on the [CRD-Fusion](https://github.com/fanxiule/CRD_Fusion/tree/custom_data) repository.

Model weights obtained after fine tuning are already saved in the `crd_fusion_perception` package.

## Running the Controller

Reference disparity image and occlusion mask recorded at the robot's target pose are saved in `visual_servo/target`. The controller can then be run by using these two images as the reference signal to regulate the robot towards the target pose.

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
This command will run the controller by using the target images as the reference signals.