#!/usr/bin/env python

import os

import rospy
import rospkg
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from depth_visual_servo_common.image_np_conversion import image_to_numpy

np.seterr(divide='ignore')  # suppress warnings from division by 0


class Converter:
    def __init__(self, baseline, focal_length, save_path):
        """
        A class to save the camera images

        :param baseline: camera baseline in mm
        :param focal_length: camera focal length in x direction in pixels
        :param save_path: directory to save the images
        """
        self.baseline = baseline
        self.focal_length = focal_length
        self.crop_w, self.crop_h = rospy.get_param(
            "/visual_servo/cropped_img_sz")
        self.bridge = CvBridge()

        self.save_path = save_path
        if not os.path.exists(self.save_path):
            os.makedirs(save_path)
        self.counter = 0

        # D435 topics
        self.left_sub = Subscriber("/camera/infra1/image_raw", Image)
        self.right_sub = Subscriber("/camera/infra2/image_raw", Image)
        self.raw_disp_sub = Subscriber("/raw_disp", Image)

        # CRD Fusion topics
        self.conf_sub = Subscriber("/confidence", Image)

        # synchronize received data
        self.synch = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.raw_disp_sub, self.conf_sub], queue_size=5, slop=0.05)
        self.synch.registerCallback(self._callback)

    @staticmethod
    def _disp_encoding(disp):
        """
        Encode the raw disparity with subpixel-level accuracy into a color image. Utilize R and G channels to store
        disparity. Implementation is based on sintel development kit. .png files are more compact than .npy files

        :param disp: raw disparity
        :return: encoded disparity in the form of BGR image (default OpenCV format)
        """
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

    def _resize_img(self, img):
        """
        Crop image

        :param image: input image
        :return: resized image
        """
        shape = np.shape(img)
        h = shape[0]
        w = shape[1]
        x1 = (w - self.crop_w) // 2
        y1 = (h - self.crop_h) // 2
        x2 = x1 + self.crop_w
        y2 = y1 + self.crop_h
        img = img[y1:y2, x1:x2]
        return img

    def _callback(self, left, right, raw_disp, conf):
        """
        Callback to crop and save images

        :param left: ROS message for left IR image
        :param right: ROS message for right IR image
        :param raw_disp: ROS message for raw disparity map
        :param dep: ROS message for depth map
        """
        # convert ROS Image message to CV2/numpy format
        try:
            left_im = self.bridge.imgmsg_to_cv2(left)
            right_im = self.bridge.imgmsg_to_cv2(right)
            raw_disp = self.bridge.imgmsg_to_cv2(raw_disp)
            conf = image_to_numpy(conf)
            print(np.shape(conf))
        except CvBridgeError as e:
            print(e)

        # save path for each image
        left_im_path = os.path.join(self.save_path, "left.png")
        right_im_path = os.path.join(self.save_path, "right.png")
        raw_disp_path = os.path.join(self.save_path, "raw_disp.png")
        conf_path = os.path.join(self.save_path, "conf.png")

        # crop images
        left_im = self._resize_img(left_im)
        right_im = self._resize_img(right_im)
        raw_disp = self._resize_img(raw_disp)

        # process disparity
        raw_disp = np.clip(raw_disp, a_min=0, a_max=192)
        raw_disp = self._disp_encoding(raw_disp)

        # save images
        cv2.imwrite(left_im_path, left_im)
        cv2.imwrite(right_im_path, right_im)
        cv2.imwrite(raw_disp_path, raw_disp)
        cv2.imwrite(conf_path, conf)

        # print number of frames processed
        print("Processed D435 frame #%d" % self.counter)
        self.counter += 1


def main(baseline, focal_length, save_path):
    rospy.init_node("save_img", anonymous=True)
    converter = Converter(baseline, focal_length, save_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("turtlebot3_d435")
    save_path = "/home/xfan/imgs"
    baseline = 1000 * rospy.get_param("/visual_servo/baseline")
    focal_length = rospy.get_param("/visual_servo/focal_fx")

    print("Use parameters: b = %.3f mm, f = %.3f px" %
          (baseline, focal_length))
    print("Save images in %s" % save_path)

    main(baseline, focal_length, save_path)
