#!/usr/bin/env python

import os
import time
import rospy
import rospkg
import numpy as np
import scipy.ndimage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from message_filters import Subscriber, ApproximateTimeSynchronizer
from depth_visual_servo_common.image_np_conversion import image_to_numpy


class DispController:
    def __init__(self, ref_disp, ref_occ, gain, occ_thres, baseline, fx, fy, img_w, img_h, cu, cv, occ_aware=True):
        self.gain = gain
        self.occ_aware = occ_aware
        self.occ_thres = occ_thres
        self.fx = fx
        self.fy = fy
        self.baseline = baseline

        # flatten ref_disp and create binary vector for ref_occ
        self.ref_disp = ref_disp.flatten()
        ref_occ_vec = ref_occ.flatten()
        self.ref_occ_mask = ref_occ_vec >= self.occ_thres

        # create pixel grids
        u = np.arange(1, img_w + 1)
        v = np.arange(1, img_h + 1)
        u, v = np.meshgrid(u, v)
        self.x = (u - cu) / self.fx
        self.y = (v - cv) / self.fy

        # subscribers and publisher
        self.disp_sub = Subscriber("/refined_disp", Image)
        self.occ_sub = Subscriber("/occlusion", Image)
        self.sub_synch = ApproximateTimeSynchronizer(
            [self.disp_sub, self.occ_sub], queue_size=2, slop=0.1)
        self.sub_synch.registerCallback(self._callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)

    def _cal_control_action(self, disp, occ):
        disp_x_grad = scipy.ndimage.sobel(disp, 1, mode='nearest')
        disp_y_grad = scipy.ndimage.sobel(disp, 0, mode='nearest')
        
        A = self.fx * disp_x_grad
        B = self.fy * disp_y_grad
        
        L_dv_x = A
        L_dv_y = B
        L_dv_z = disp - self.x * A - self.y * B

        L_dw_x = self.y * disp - self.x * self.y * A - (1 + self.y ** 2) * B
        L_dw_y = -self.x * disp + (1 + self.x ** 2) * A + self.x * self.y * B
        L_dw_z = self.x * B - self.y * A

        L_dv_x = L_dv_x.flatten()
        L_dv_y = L_dv_y.flatten()
        L_dv_z = L_dv_z.flatten()
        L_dw_x = L_dw_x.flatten()
        L_dw_y = L_dw_y.flatten()
        L_dw_z = L_dw_z.flatten()
        disp_control = disp.flatten()
        occ_control = occ.flatten()
        
        curr_occ_mask = occ_control >= self.occ_thres
        occ_selection = curr_occ_mask * self.ref_occ_mask  # equivalent to and operator

        ref_disp = self.ref_disp[occ_selection]
        disp_control = disp_control[occ_selection]
        occ_control = occ_control[occ_selection]
        L_dv_x = L_dv_x[occ_selection]
        L_dv_y = L_dv_y[occ_selection]
        L_dv_z = L_dv_z[occ_selection]
        L_dw_x = L_dw_x[occ_selection]
        L_dw_y = L_dw_y[occ_selection]
        L_dw_z = L_dw_z[occ_selection]

        L_dv = np.stack((L_dv_x, L_dv_y, L_dv_z), axis=-1)
        L_dw = np.stack((L_dw_x, L_dw_y, L_dw_z), axis=-1)
        L_dv_factor = disp_control / (self.baseline * self.fx)
        L_dv = np.expand_dims(L_dv_factor, -1) * L_dv
        L_d = np.concatenate((L_dv, L_dw), axis=-1)
        L_d_inv = np.linalg.pinv(L_d)
        vel = - self.gain * np.matmul(L_d_inv, (disp_control - ref_disp))
        return vel


    def _callback(self, disp_msg, occ_msg):
        disp = image_to_numpy(disp_msg)
        occ = image_to_numpy(occ_msg) / 255.0

        cmd_vel = self._cal_control_action(disp, occ)

        vel = Twist()
        vel.linear.x = cmd_vel[2]
        vel.angular.z = - cmd_vel[4]
        print(vel)
        self.vel_pub.publish(vel)
        print(time.time())


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("visual_servo")
    save_folder = os.path.join(pkg_path, "target")
    ref_disp = np.load(os.path.join(save_folder, "disp.npy"))
    ref_occ = np.load(os.path.join(save_folder, "occ.npy"))
    occ_aware = True
    gain = 70
    occ_thres = 0.9
    # TODO
    fx = 435
    fy = 435
    img_w = 320
    img_h = 240
    cu = img_w // 2
    cv = img_h // 2
    baseline = 0.05

    rospy.init_node("controller", anonymous=True)
    controller = DispController(ref_disp, ref_occ, gain, occ_thres, baseline, fx, fy, img_w, img_h, cu, cv, occ_aware)

    try:
        rospy.spin()
        # rate.sleep()
    except KeyboardInterrupt:
        print("Shut down")
