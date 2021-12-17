#!/usr/bin/env python3

import os
import time
import sys
import numpy as np
import rospy
import torch
import rospkg
from rospy.topics import Subscriber

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from image_np_conversion import image_to_numpy, numpy_to_image

from conf_generation import ConfGeneration
from crd_fusion import CRDFusionNet


class DispRefiner:
    def __init__(self, checkpt, device):
        self.device = device

        # input images are 1280 x 720
        # crop them to 640 x 480 to be consistent with image size for CRD-Fusion training
        crop_w = 320
        crop_h = 240
        self.crop_x1 = (800 - crop_w) // 2
        self.crop_y1 = (600 - crop_h) // 2
        self.crop_x2 = self.crop_x1 + crop_w
        self.crop_y2 = self.crop_y1 + crop_h
        self.max_disp = 192
        self.conf_thres = 0.8

        self.conf_gen = ConfGeneration(self.device, True)
        self.network = CRDFusionNet(
            [0, 1, 2, 3], self.max_disp, crop_h, crop_w, False, True)
        self.network.load_model(checkpt)
        self.network.to(self.device)

        self.imgnet_mean = torch.tensor([0.485, 0.456, 0.406]).unsqueeze(
            0).unsqueeze(-1).unsqueeze(-1)
        self.imgnet_std = torch.tensor([0.229, 0.224, 0.225]).unsqueeze(
            0).unsqueeze(-1).unsqueeze(-1)
        self.imgnet_mean = self.imgnet_mean.to(self.device)
        self.imgnet_std = self.imgnet_std.to(self.device)

        self.left_img_sub = Subscriber("/camera/infra1/image_raw", Image)
        self.right_img_sub = Subscriber("/camera/infra2/image_raw", Image)
        self.raw_disp_sub = Subscriber("/raw_disp", Image)
        # TODO slop needs to be lower -> use less expensive raw disp stereo matching
        self.sub_synch = ApproximateTimeSynchronizer(
            [self.left_img_sub, self.right_img_sub, self.raw_disp_sub], queue_size=2, slop=0.1)
        self.sub_synch.registerCallback(self._callback)
        self.refined_disp_pub = rospy.Publisher("/refined_disp", Image, queue_size=10)
        self.occ_pub = rospy.Publisher("/occlusion", Image, queue_size=10)

    def _norm_disp(self, disp):
        norm_disp = torch.clamp(disp, min=0, max=self.max_disp)
        norm_disp = norm_disp / self.max_disp
        return norm_disp

    def _np_img2tensor(self, img):
        img = torch.from_numpy(img / 255.0).unsqueeze(0).unsqueeze(0)
        img = torch.cat((img, img, img), dim=1).to(
            torch.float32).to(self.device)
        img = (img - self.imgnet_mean) / self.imgnet_std
        return img

    def _callback(self, left, right, raw_d):
        left_ir = image_to_numpy(left)
        right_ir = image_to_numpy(right)
        raw_disp = image_to_numpy(raw_d)

        left_ir = left_ir[self.crop_y1:self.crop_y2, self.crop_x1:self.crop_x2]
        right_ir = right_ir[self.crop_y1:self.crop_y2,
                            self.crop_x1:self.crop_x2]
        raw_disp = raw_disp[self.crop_y1:self.crop_y2,
                            self.crop_x1:self.crop_x2]

        # np.save("/home/xfan/Desktop/left.npy", left_ir)
        # np.save("/home/xfan/Desktop/right.npy", right_ir)
        # np.save("/home/xfan/Desktop/disp.npy", raw_disp)

        left_ir = self._np_img2tensor(left_ir)
        right_ir = self._np_img2tensor(right_ir)
        raw_disp = torch.from_numpy(raw_disp).unsqueeze(
            0).unsqueeze(0).to(self.device)
        norm_raw_disp = self._norm_disp(raw_disp)
        conf = self.conf_gen.cal_confidence(left_ir, right_ir, raw_disp)
        conf[conf < self.conf_thres] = 0
        outputs = self.network(left_ir, right_ir, norm_raw_disp, conf)
        
        refined_disp = torch.squeeze(outputs['refined_disp0']).detach().cpu().numpy()
        occ = torch.squeeze(outputs['occ0']).detach().cpu().numpy()
        occ = (255*occ).astype("uint8")
        refined_disp_msg = numpy_to_image(refined_disp, "32FC1")
        occ_msg = numpy_to_image(occ, "8UC1")
        self.refined_disp_pub.publish(refined_disp_msg)
        self.occ_pub.publish(occ_msg)
        print(time.time())


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("crd_fusion_perception")
    checkpt = os.path.join(pkg_path, "checkpt")
    device = "cuda"
    if len(sys.argv) == 2 and sys.argv[1] == "cpu":
        device = sys.argv[1]
    if not os.path.exists(checkpt) or (device != "cuda" and device != "cpu"):
        print("Either checkpoint is not found or passed in wrong device")
        raise RuntimeError

    rospy.init_node("disp_refiner", anonymous=True)
    disp_refiner = DispRefiner(checkpt, device)
    # rate = rospy.Rate(10)
    try:
        rospy.spin()
        # rate.sleep()
    except KeyboardInterrupt:
        print("Shut down")
