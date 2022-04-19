#!/usr/bin/env python

import os
import rospy
import rospkg
import numpy as np
from tf.transformations import euler_from_quaternion
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from depth_visual_servo_common.image_np_conversion import image_to_numpy


class OdomProcessor:
    def __init__(self, save_path):
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        self.state_file = os.path.join(save_path, "robot_state.txt")
        open(self.state_file, "w").close()
        odom_sub = rospy.Subscriber("/odom", Odometry, self._callback)

    def _callback(self, odom):
        sec = odom.header.stamp.secs
        nsec = odom.header.stamp.nsecs
        time = 1.0 * sec + 1.0 * nsec / (10**9)

        pos_x = odom.pose.pose.position.x
        pos_y = odom.pose.pose.position.y
        quat_x = odom.pose.pose.orientation.x
        quat_y = odom.pose.pose.orientation.y
        quat_z = odom.pose.pose.orientation.z
        quat_w = odom.pose.pose.orientation.w
        orient_z = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])[
            2] * 180 / 3.14159

        v = odom.twist.twist.linear.x
        omega = odom.twist.twist.angular.z * 180 / 3.14159

        str_to_save = "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (
            time, pos_x, pos_y, orient_z, v, omega)
        with open(self.state_file, "a") as f:
            f.write(str_to_save)


class ImageProcessor:
    def __init__(self, save_path, ref_path):
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        self.disp_path = os.path.join(save_path, "disp")
        self.occ_path = os.path.join(save_path, "occ")
        if not os.path.exists(self.disp_path):
            os.makedirs(self.disp_path)
        if not os.path.exists(self.occ_path):
            os.makedirs(self.occ_path)

        self.ref_disp = np.load(os.path.join(ref_path, "disp.npy"))
        self.ref_occ = np.load(os.path.join(ref_path, "occ.npy"))

        self.task_file = os.path.join(save_path, "task_err.txt")
        open(self.task_file, "w").close()

        disp_sub = Subscriber("/refined_disp", Image)
        occ_sub = Subscriber("/occlusion", Image)
        synch = ApproximateTimeSynchronizer(
            [disp_sub, occ_sub], queue_size=10, slop=0.05)
        synch.registerCallback(self._callback)

        self.initial_frame = True
    
    def _callback(self, disp, occ):
        sec = disp.header.stamp.secs
        nsec = disp.header.stamp.nsecs
        time = 1.0 * sec + 1.0 * nsec / (10**9)
        disp = image_to_numpy(disp)
        occ = image_to_numpy(occ) / 255.0
        total_px = disp.size

        task_err = (self.ref_occ * occ) * np.abs(self.ref_disp - disp)
        task_err = np.sum(task_err) / total_px
        
        if self.initial_frame:
            disp_file = os.path.join(self.disp_path, "init.npy")
            occ_file = os.path.join(self.occ_path, "init.npy")
            self.initial_frame = False
        else:
            disp_file = os.path.join(self.disp_path, "final.npy")
            occ_file = os.path.join(self.occ_path, "final.npy")
        np.save(disp_file, disp)
        np.save(occ_file, occ)
        
        with open(self.task_file, "a") as f:
            f.write("%.6f,%.5f\n" % (time, task_err))


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("visual_servo")
    save_path = os.path.join(pkg_path, "robot_state")
    ref_path = os.path.join(pkg_path, "target")

    rospy.init_node("rosbag_processor", anonymous=True)
    odom_processor = OdomProcessor(save_path)
    img_processor = ImageProcessor(save_path, ref_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")