#!/usr/bin/env python

import os
import rospy
import rospkg
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image

from depth_visual_servo_common.image_np_conversion import image_to_numpy


class DatasetConverter:
    def __init__(self, baseline, focal_length, save_path):
        self.baseline = baseline
        self.focal_length = focal_length
        self.counter = 0
        self.left_path = os.path.join(save_path, "left")
        self.right_path = os.path.join(save_path, "right")
        self.disp_path = os.path.join(save_path, "raw_disp")
        self.gt_path = os.path.join(save_path, "gt")
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            os.makedirs(self.left_path)
            os.makedirs(self.right_path)
            os.makedirs(self.disp_path)
            os.makedirs(self.gt_path)

        left_sub = Subscriber("/camera/infra1/image_raw", Image)
        right_sub = Subscriber("/camera/infra2/image_raw", Image)
        disp_sub = Subscriber("/raw_disp", Image)
        gt_sub = Subscriber("/camera/depth/image_raw", Image)
        synch = ApproximateTimeSynchronizer(
            [left_sub, right_sub, disp_sub, gt_sub], queue_size=10, slop=0.05)
        synch.registerCallback(self._callback)

    @staticmethod
    def _disp_encoding(disp):
        disp[disp > 1024] = 1024
        disp[disp < 0] = 0
        encoded_disp = np.zeros(
            (disp.shape[0], disp.shape[1], 3), dtype='uint8')
        d_r, remainder = np.divmod(disp, 4.0)
        d_r = d_r.astype('uint8')
        d_g = (64.0 * remainder).astype('uint8')
        # Note that OpenCV by default treats image as BGR instead of RGB
        encoded_disp[:, :, 1] = d_g
        encoded_disp[:, :, 2] = d_r
        return encoded_disp

    def _callback(self, left, right, disp, gt):
        left_ir = image_to_numpy(left)
        right_ir = image_to_numpy(right)
        raw_disp = image_to_numpy(disp)
        gt_depth = image_to_numpy(gt) / 1000.0  # from mm to m

        gt_disp = self.baseline * self.focal_length / gt_depth
        gt_disp = np.nan_to_num(gt_disp)

        raw_disp = self._disp_encoding(raw_disp)
        gt_disp = self._disp_encoding(gt_disp)

        left_img_path = os.path.join(self.left_path, "%d.png" % self.counter)
        right_img_path = os.path.join(self.right_path, "%d.png" % self.counter)
        raw_disp_path = os.path.join(self.disp_path, "%d.png" % self.counter)
        gt_disp_path = os.path.join(self.gt_path, "%d.png" % self.counter)

        cv2.imwrite(left_img_path, left_ir)
        cv2.imwrite(right_img_path, right_ir)
        cv2.imwrite(raw_disp_path, raw_disp)
        cv2.imwrite(gt_disp_path, gt_disp)

        print("Processed frame #%d" % self.counter)
        self.counter += 1


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("depth_visual_servo_common")
    save_path = os.path.join(pkg_path, "realsense")

    baseline = 0.05
    focal_length = 435
    rospy.init_node("dataset_converter", anonymous=True)
    converter = DatasetConverter(baseline, focal_length, save_path)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
