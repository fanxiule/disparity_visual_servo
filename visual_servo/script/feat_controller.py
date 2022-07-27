#!/usr/bin/env python3

import os
import time

import rospy
import rospkg
import cv2
import skimage.io
import numpy as np
from skimage.feature import match_descriptors, ORB
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from depth_visual_servo_common.image_np_conversion import image_to_numpy


class FeatController:
    def __init__(self, ref_disp, ref_left, gain, baseline, fx, fy, cu, cv, img_h, img_w, orig_h, orig_w, CRD_flag, ORB_flag=True):
        self.gain = gain
        self.baseline = baseline
        self.fx = fx
        self.fy = fy
        self.cu = cu
        self.cv = cv

        self.crop_x1 = (orig_w - img_w) // 2
        self.crop_y1 = (orig_h - img_h) // 2
        self.crop_x2 = self.crop_x1 + img_w
        self.crop_y2 = self.crop_y1 + img_h

        self.ref_depth = self.baseline * self.fx / ref_disp
        self.ORB_flag = ORB_flag
        self.CRD_flag = CRD_flag
        if self.ORB_flag:
            self.descriptor_extractor = ORB(n_keypoints=500)
            self.descriptor_extractor.detect_and_extract(ref_left)
            self.keypoints_ref = self.descriptor_extractor.keypoints
            self.descriptors_ref = self.descriptor_extractor.descriptors
        else:
            self.descriptor_extractor = cv2.SIFT_create(nfeatures=500)
            keypoints_ref, self.descriptors_ref = self.descriptor_extractor.detectAndCompute(ref_left, None)
            self.keypoints_ref = np.zeros((len(keypoints_ref), 2))
            for i in range(len(keypoints_ref)):
                self.keypoints_ref[i, :] = np.asarray([keypoints_ref[i].pt[1], keypoints_ref[i].pt[0]])

        # subscribers and publisher
        if self.CRD_flag:
            self.disp_sub = Subscriber("/refined_disp", Image)
        else:
            self.disp_sub = Subscriber("/raw_disp", Image)
        self.ir_sub = Subscriber("/camera/infra1/image_raw", Image)
        self.sub_synch = ApproximateTimeSynchronizer(
            [self.disp_sub, self.ir_sub], queue_size=1, slop=0.05)
        self.sub_synch.registerCallback(self._callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # a different strategy for camera-wheel velocity transformation
        self.J = np.array([[0, -0.045], [1, -0.018], [0, -1]])
        self.J = np.linalg.pinv(self.J)

        self.start_time = None
        self.total_steps = 0

    def _feat_matching(self, ir):
        if self.ORB_flag:
            self.descriptor_extractor.detect_and_extract(ir)
            keypoints_curr = self.descriptor_extractor.keypoints
            descriptors_curr = self.descriptor_extractor.descriptors
        else:
            keypoints_curr_, descriptors_curr = self.descriptor_extractor.detectAndCompute(ir, None)
            keypoints_curr = np.zeros((len(keypoints_curr_), 2))
            for i in range(len(keypoints_curr_)):
                keypoints_curr[i, :] = np.asarray([keypoints_curr_[i].pt[1], keypoints_curr_[i].pt[0]])
        matches = match_descriptors(
            descriptors_curr, self.descriptors_ref, max_ratio=0.7, cross_check=True)
        matched_px_curr = keypoints_curr[matches[:, 0], :]
        matched_px_ref = self.keypoints_ref[matches[:, 1], :]
        return matched_px_curr, matched_px_ref

    def _cal_control_action(self, matched_px_curr, matched_px_ref, curr_depth):
        matched_px_curr_u = matched_px_curr[:, 1]
        matched_px_curr_v = matched_px_curr[:, 0]
        matched_px_ref_u = matched_px_ref[:, 1]
        matched_px_ref_v = matched_px_ref[:, 0]

        matched_px_curr_depth = curr_depth[matched_px_curr_v.astype(
            np.int), matched_px_curr_u.astype(np.int)]
        # matched_px_ref_depth = self.ref_depth[matched_px_ref_v.astype(np.int), matched_px_ref_u.astype(np.int)]

        matched_px_curr_x = (matched_px_curr_u - self.cu) / self.fx
        matched_px_curr_y = (matched_px_curr_v - self.cv) / self.fy
        matched_px_ref_x = (matched_px_ref_u - self.cu) / self.fx
        matched_px_ref_y = (matched_px_ref_v - self.cv) / self.fy
        
        matched_px_curr_xy = np.stack((matched_px_curr_x, matched_px_curr_y))
        matched_px_curr_xy = np.transpose(matched_px_curr_xy)
        matched_px_curr_xy = matched_px_curr_xy.flatten()
        matched_px_ref_xy = np.stack((matched_px_ref_x, matched_px_ref_y))
        matched_px_ref_xy = np.transpose(matched_px_ref_xy)
        matched_px_ref_xy = matched_px_ref_xy.flatten()
        
        Lxx_curr_11 = - 1 / matched_px_curr_depth
        Lxx_curr_12 = matched_px_curr_x / matched_px_curr_depth
        Lxx_curr_13 = - (1 + matched_px_curr_x ** 2)

        Lxy_curr_21 = np.zeros_like(Lxx_curr_11)
        Lxy_curr_22 = matched_px_curr_y / matched_px_curr_depth
        Lxy_curr_23 = - matched_px_curr_x * matched_px_curr_y
        
        Lx_curr = np.stack(
            (Lxx_curr_11, Lxx_curr_12, Lxx_curr_13, Lxy_curr_21, Lxy_curr_22, Lxy_curr_23))
        Lx_curr = np.transpose(Lx_curr)
        Lx_curr = np.reshape(Lx_curr, (-1, 3))
        Lx_curr_inv = np.linalg.pinv(Lx_curr)
        vel = - self.gain * \
            np.matmul(Lx_curr_inv, (matched_px_curr_xy - matched_px_ref_xy))
        vel = np.matmul(self.J, vel)
        return vel

    def _callback(self, disp_msg, ir_msg):
        if self.start_time is None:
            self.start_time = time.time()

        disp = image_to_numpy(disp_msg)
        if not self.CRD_flag:
            disp = disp[self.crop_y1:self.crop_y2,
                        self.crop_x1:self.crop_x2]
        depth = self.baseline * self.fx / disp
        ir = image_to_numpy(ir_msg)
        ir = ir[self.crop_y1:self.crop_y2,
                self.crop_x1:self.crop_x2]
        matched_px_curr, matched_px_ref = self._feat_matching(ir)
        cmd_vel = self._cal_control_action(matched_px_curr, matched_px_ref, depth)

        # publish robot velocity
        vel = Twist()
        vel.linear.x = cmd_vel[0]
        vel.angular.z = cmd_vel[1]
        self.vel_pub.publish(vel)
        self.total_steps += 1
        print("FPS: %.2f" % (self.total_steps / (time.time() - self.start_time)))


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("visual_servo")

    # Controller setting
    crd_fusion = False
    gain = 0.1
    ORB_flag = True

    # Load target
    save_folder = os.path.join(pkg_path, "target")
    ref_left = skimage.io.imread(os.path.join(save_folder, "left.png"))
    if crd_fusion:
        ref_disp = np.load(os.path.join(save_folder, "disp.npy"))
    else:
        ref_disp = np.load(os.path.join(save_folder, "raw_disp.npy"))

    # camera settings
    fx = rospy.get_param("/visual_servo/focal_fx")
    fy = rospy.get_param("/visual_servo/focal_fy")
    img_w, img_h = rospy.get_param("/visual_servo/cropped_img_sz")
    orig_w, orig_h = rospy.get_param("/visual_servo/orig_img_sz")
    cu, cv = rospy.get_param("/visual_servo/cropped_prinicipal_pt")
    baseline = rospy.get_param("/visual_servo/baseline")

    rospy.init_node("controller", anonymous=True)
    controller = FeatController(
        ref_disp, ref_left, gain, baseline, fx, fy, cu, cv, img_h, img_w, orig_h, orig_w, crd_fusion, ORB_flag)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
