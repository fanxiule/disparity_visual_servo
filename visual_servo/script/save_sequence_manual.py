#!/usr/bin/env python

import os
import rospy
import rospkg
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image

from depth_visual_servo_common.image_np_conversion import image_to_numpy


# Save the target images in the sequence/ folder one by one for the navigation experiment 

class DispConverter:
    def __init__(self, save_path, save_id):
        self.save_path = save_path
        self.save_id = save_id
        self.frame_saved_flag = False
        self.disp_path = os.path.join(save_path, "disp")
        self.disp_img_path = os.path.join(save_path, "disp_img")
        self.occ_path = os.path.join(save_path, "occ")
        self.occ_img_path = os.path.join(save_path, "occ_img")
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            os.makedirs(self.disp_path)
            os.makedirs(self.disp_img_path)
            os.makedirs(self.occ_path)
            os.makedirs(self.occ_img_path)
        
        disp_sub = Subscriber("/refined_disp", Image)
        occ_sub = Subscriber("/occlusion", Image)
        disp_synch = ApproximateTimeSynchronizer([disp_sub, occ_sub], queue_size=1, slop=0.05)
        disp_synch.registerCallback(self._disp_callback)
    
    def _disp_callback(self, disp, occ):
        if not self.frame_saved_flag:
            refined_disp = image_to_numpy(disp)
            occlusion = image_to_numpy(occ) / 255.0
            ref_disp_path = os.path.join(self.disp_path, "%d.npy" % self.save_id)
            ref_disp_img_path = os.path.join(self.disp_img_path, "%d.png" % self.save_id)
            occlusion_path = os.path.join(self.occ_path, "%d.npy" % self.save_id)
            occlusion_img_path = os.path.join(self.occ_img_path, "%d.png" % self.save_id)

            np.save(ref_disp_path, refined_disp)
            np.save(occlusion_path, occlusion)
            refined_disp_img = refined_disp.astype(np.uint8)
            occlusion_img = (occlusion *  255).astype(np.uint8)
            cv2.imwrite(ref_disp_img_path, refined_disp_img)
            cv2.imwrite(occlusion_img_path, occlusion_img)

            self.frame_saved_flag = True
            print("Saved disparity ID# %d" % self.save_id)        

class StereoConverter:
    def __init__(self, save_path, save_id):
        self.save_path = save_path
        self.save_id = save_id
        self.frame_saved_flag = False
        self.left_path = os.path.join(save_path, "left")
        self.right_path = os.path.join(save_path, "right")
        if not os.path.exists(self.left_path):
            os.makedirs(self.left_path)
            os.makedirs(self.right_path)
        
        left_sub = Subscriber("/camera/infra1/image_rect_raw", Image)
        right_sub = Subscriber("/camera/infra2/image_rect_raw", Image)
        stereo_synch = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=1, slop=0.05)
        stereo_synch.registerCallback(self._stereo_callback)
    
    def _stereo_callback(self, left, right):
        if not self.frame_saved_flag:
            left_ir = image_to_numpy(left)
            right_ir = image_to_numpy(right)
            left_ir_path = os.path.join(self.left_path, "%d.png" % self.save_id)
            right_ir_path = os.path.join(self.right_path, "%d.png" % self.save_id)
            cv2.imwrite(left_ir_path, left_ir)
            cv2.imwrite(right_ir_path, right_ir)

            self.frame_saved_flag = True
            print("Saved IR ID# %d" % self.save_id)


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("visual_servo")
    save_path = os.path.join(pkg_path, "sequence")
    disp_path = os.path.join(save_path, "disp")
    if os.path.exists(disp_path):
        id_list = os.listdir(disp_path)
        if len(id_list) == 0:
            save_id = 0
        else:
            for i in range(len(id_list)):
                id = id_list[i]
                id = id.strip(".npy")
                id_list[i] = int(id)
            id_list.sort()
            save_id = id_list[-1] + 1
    else:
        save_id = 0
    rospy.init_node("manual_sequence_converter", anonymous=True)
    disp_converter = DispConverter(save_path, save_id)
    stereo_converter = StereoConverter(save_path, save_id)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
