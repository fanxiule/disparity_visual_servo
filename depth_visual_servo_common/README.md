# Common Files for the Disparity-based Visual Servoing Algorithm

This package containes some common and miscellaneous code to run the controller. There are three launch files in this package:

- `collect_dataset.launch`: for collecting a dataset in the simulation environment
- `convert_dataset.launch`: for converting the collected dataset from a rosbag file to images
- `set_common_param.launch`: for setting some common variables (e.g., focal length, baseline, image size, etc.) used by different processes in the algorithm

## Acknowledgement

We would like to thank [this repository](https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/image.py) for providing the conversion between `sensor_msgs/Image` and numpy arrays.