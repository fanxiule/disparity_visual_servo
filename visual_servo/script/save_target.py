#!/usr/bin/env python

import os
import cv2
import numpy as np
import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from depth_visual_servo_common.image_np_conversion import image_to_numpy

# Ensure the robot is stationary when executing this script


rospack = rospkg.RosPack()
pkg_path = rospack.get_path("visual_servo")
save_folder = os.path.join(pkg_path, "target")
if not os.path.exists(save_folder):
    os.makedirs(save_folder)
# Use flag to save the target information only once
odom_saved = False
occ_saved = False
disp_saved = False


def odom_callback(odom):
    """
    Call back to save the robot's odometry

    :param odom: ROS message with robot's odometry
    """
    global odom_saved
    if not odom_saved:
        # pose information
        pose = odom.pose.pose
        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z
        orient_x = pose.orientation.x
        orient_y = pose.orientation.y
        orient_z = pose.orientation.z
        orient_w = pose.orientation.w

        pose_str = "pos_x: %.3f\npos_y: %.3f\npos_z: %.3f\norient_x: %.3f\norient_y: %.3f\norient_z: %.3f\norient_w: %.3f" % (
            pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w)

        # write pose to txt file
        with open(os.path.join(save_folder, "pose.txt"), "w") as f:
            f.write(pose_str)

        print("Pose saved")
        odom_saved = True


def disp_callback(disp):
    """
    Callback to save the target disparity

    :param disp: ROS message with the disparity map
    """
    global disp_saved
    if not disp_saved:
        disp_np = image_to_numpy(disp)
        disp_np = np.squeeze(disp_np)
        np.save(os.path.join(save_folder, "disp.npy"), disp_np)
        cv2.imwrite(os.path.join(save_folder, "disp.png"), (255 * disp_np / 192).astype(np.uint8))
        print("Disparity saved")
        disp_saved = True


def occ_callback(occ):
    """
    Callback to save the target occlusion mask
    
    :param occ: ROS message with the occlusion mask
    """
    global occ_saved
    if not occ_saved:
        occ_np = image_to_numpy(occ)
        occ_np = np.squeeze(occ_np) / 255.0  # occlusion mask in uint8, convert it to [0, 1]
        np.save(os.path.join(save_folder, "occ.npy"), occ_np)
        cv2.imwrite(os.path.join(save_folder, "occ.png"), (255 * occ_np).astype(np.uint8))
        print("Occlusion saved")
        occ_saved = True


if __name__ == "__main__":
    rospy.init_node("save_target", anonymous=True)

    odom_subscriber = rospy.Subscriber("/odom", Odometry, odom_callback)
    disp_subscriber = rospy.Subscriber("/refined_disp", Image, disp_callback)
    occ_subscriber = rospy.Subscriber("/occlusion", Image, occ_callback)

    rospy.spin()
