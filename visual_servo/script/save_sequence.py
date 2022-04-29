#!/usr/bin/env python

import os
import rospy
import rospkg
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image

from depth_visual_servo_common.image_np_conversion import image_to_numpy


class DispConverter:
    def __init__(self, save_path, step_sz):
        self.save_path = save_path
        self.step_sz = step_sz
        self.iter_id = 0
        self.step_id = 0
        self.init_filled = False
        self.disp_path = os.path.join(save_path, "disp")
        self.occ_path = os.path.join(save_path, "occ")
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            os.makedirs(self.disp_path)
            os.makedirs(self.occ_path)
        
        disp_sub = Subscriber("/refined_disp", Image)
        occ_sub = Subscriber("/occlusion", Image)
        disp_synch = ApproximateTimeSynchronizer([disp_sub, occ_sub], queue_size=1, slop=0.05)
        disp_synch.registerCallback(self._disp_callback)
    
    def _disp_callback(self, disp, occ):
        refined_disp = image_to_numpy(disp)
        occlusion = image_to_numpy(occ) / 255.0
        
        if self.iter_id % self.step_sz == 0:
            ref_disp_path = os.path.join(self.disp_path, "%d.npy" % self.step_id)
            occlusion_path = os.path.join(self.occ_path, "%d.npy" % self.step_id)

            np.save(ref_disp_path, refined_disp)
            np.save(occlusion_path, occlusion)

            print("Processed frame #%d" % self.step_id)        
            self.step_id += 1
        self.iter_id += 1

class StereoConverter:
    def __init__(self, save_path):
        self.save_path = save_path
        self.init_filled = False
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
        left_ir = image_to_numpy(left)
        right_ir = image_to_numpy(right)

        if not self.init_filled:
            left_ir_path = os.path.join(self.left_path, "init.png")
            right_ir_path = os.path.join(self.right_path, "init.png")
             
            cv2.imwrite(left_ir_path, left_ir)
            cv2.imwrite(right_ir_path, right_ir)
            
            self.init_filled = True
        else:
            left_ir_path = os.path.join(self.left_path, "final.png")
            right_ir_path = os.path.join(self.right_path, "final.png")
             
            cv2.imwrite(left_ir_path, left_ir)
            cv2.imwrite(right_ir_path, right_ir)


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("visual_servo")
    save_path = os.path.join(pkg_path, "sequence")
    step_sz = 10
    rospy.init_node("sequence_converter", anonymous=True)
    disp_converter = DispConverter(save_path, step_sz)
    stereo_converter = StereoConverter(save_path)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
