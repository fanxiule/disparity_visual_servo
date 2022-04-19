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
raw_disp_saved = False
left_saved = False
right_saved = False

crop_w, crop_h = rospy.get_param("/visual_servo/cropped_img_sz")
orig_w, orig_h = rospy.get_param("/visual_servo/orig_img_sz")
crop_x1 = (orig_w - crop_w) // 2
crop_y1 = (orig_h - crop_h) // 2
crop_x2 = crop_x1 + crop_w
crop_y2 = crop_y1 + crop_h
baseline = 1000 * rospy.get_param("/visual_servo/baseline")
fx = rospy.get_param("/visual_servo/focal_fx")

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


def raw_disp_callback(raw_disp):
    global raw_disp_saved, crop_x1, crop_x2, crop_y1, crop_y2, baseline, fx
    if not raw_disp_saved:
        raw_disp_np = image_to_numpy(raw_disp)
        raw_disp_np = np.squeeze(raw_disp_np)
        raw_disp_np = baseline * fx / raw_disp_np
        raw_disp_np = np.nan_to_num(raw_disp_np)
        raw_disp_np[raw_disp_np >= 192.0] = 0.0
        raw_disp_np = raw_disp_np[crop_y1:crop_y2,
                                  crop_x1:crop_x2]
        np.save(os.path.join(save_folder, "raw_disp.npy"), raw_disp_np)
        cv2.imwrite(os.path.join(save_folder, "raw_disp.png"),
                    (255 * raw_disp_np / 192).astype(np.uint8))
        print("Raw disparity saved")
        raw_disp_saved = True


def left_callback(left):
    global left_saved, crop_x1, crop_x2, crop_y1, crop_y2
    if not left_saved:
        left_np = image_to_numpy(left)
        left_np = np.squeeze(left_np)
        left_np = left_np[crop_y1:crop_y2,
                          crop_x1:crop_x2]
        cv2.imwrite(os.path.join(save_folder, "left.png"),
                    left_np.astype(np.uint8))
        print("Left stereo image saved")
        left_saved = True


def right_callback(right):
    global right_saved, crop_x1, crop_x2, crop_y1, crop_y2
    if not right_saved:
        right_np = image_to_numpy(right)
        right_np = np.squeeze(right_np)
        right_np = right_np[crop_y1:crop_y2,
                            crop_x1:crop_x2]
        cv2.imwrite(os.path.join(save_folder, "right.png"),
                    right_np.astype(np.uint8))
        print("right stereo image saved")
        right_saved = True

if __name__ == "__main__":
    rospy.init_node("save_target", anonymous=True)

    odom_subscriber = rospy.Subscriber("/odom", Odometry, odom_callback)
    disp_subscriber = rospy.Subscriber("/refined_disp", Image, disp_callback)
    occ_subscriber = rospy.Subscriber("/occlusion", Image, occ_callback)
    raw_disp_subscriber = rospy.Subscriber(
        "/camera/depth/image_rect_raw", Image, raw_disp_callback)
    left_subscriber = rospy.Subscriber(
        "/camera/infra1/image_rect_raw", Image, left_callback)
    right_subscriber = rospy.Subscriber(
        "/camera/infra2/image_rect_raw", Image, right_callback)

    rospy.spin()
