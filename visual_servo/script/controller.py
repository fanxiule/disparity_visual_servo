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


# p^b_o = [0.045, 0.018, 0.104]
# Moving axis rotation: xb>-90, yb'>+90
# R^b_o = [0, 0, 1; -1, 0, 0; 0, -1, 0]


class DispController:
    def __init__(self, ref_disp, ref_occ, gain, occ_thres, baseline, fx, fy, img_w, img_h, cu, cv, occ_aware=True):
        """
        Controller for disparity based visual servoing

        :param ref_disp: reference disparity map
        :param ref_occ: reference occlusion map
        :param gain: controller gain
        :param occ_thres: occlusion threshold for filtering
        :param baseline: camera baseline
        :param fx: focal length in x
        :param fy: focal length in y
        :param img_w: image width
        :param img_h: image height
        :param cu: principal point in x
        :param cv: principal point in y
        :param occ_aware: a flag if the controller is occlusion-aware
        """
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
        self.x = u - cu
        self.y = v - cv

        # subscribers and publisher
        self.disp_sub = Subscriber("/refined_disp", Image)
        self.occ_sub = Subscriber("/occlusion", Image)
        self.sub_synch = ApproximateTimeSynchronizer(
            [self.disp_sub, self.occ_sub], queue_size=1, slop=0.05)
        self.sub_synch.registerCallback(self._callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # a different strategy for camera-wheel velocity transformation
        self.J = np.array([[0, -0.045], [1, -0.018], [0, -1]])
        self.J = np.linalg.pinv(self.J)

        self.start_time = time.time()
        self.total_steps = 0

    def _cal_control_action(self, disp, occ):
        """
        Calculate the control action
        
        :param disp: disparity map
        :param occ: occlusion mask
        :return: calculated 6 DOF velocity
        """
        disp_x_grad = scipy.ndimage.sobel(disp, 1, mode='nearest')
        disp_y_grad = scipy.ndimage.sobel(disp, 0, mode='nearest')

        # calculate each term in the interaction matrix
        L_dv_x = disp / self.baseline * disp_x_grad
        L_dv_z = disp ** 2 / (self.baseline * self.fx) - disp * self.x * disp_x_grad / (self.baseline * self.fx) - disp * self.y * disp_y_grad / (self.baseline * self.fx)
        L_dw_y = -disp * self.x / self.fx + (self.fy ** 2 + self.x ** 2) * disp_x_grad / self.fx + self.x * self.y * disp_y_grad / self.fx

        # flatten all 2D values
        L_dv_x = L_dv_x.flatten()
        L_dv_z = L_dv_z.flatten()
        L_dw_y = L_dw_y.flatten()
        disp_control = disp.flatten()
        occ_control = occ.flatten()

        # select non-occluded pixels in both the current view and the target view
        curr_occ_mask = occ_control >= self.occ_thres
        # equivalent to or operator
        occ_selection = curr_occ_mask + self.ref_occ_mask  
        occ_selection = occ_selection > 0

        # select non-occluded pixels
        ref_disp = self.ref_disp[occ_selection]
        disp_control = disp_control[occ_selection]
        occ_control = occ_control[occ_selection]
        L_dv_x = L_dv_x[occ_selection]
        L_dv_z = L_dv_z[occ_selection]
        L_dw_y = L_dw_y[occ_selection]

        # build the final interaction matrix
        L_d = np.stack((L_dv_x, L_dv_z, L_dw_y), axis=-1)
        
        # calculate the velocity
        L_d_inv = np.linalg.pinv(L_d)
        if self.occ_aware:
            L_d_inv = occ_control * L_d_inv  # equivalent to building a diagonal matrix from occ_control
        # velocity in the camera frame
        vel = - self.gain * np.matmul(L_d_inv, (disp_control - ref_disp))
        # velocity in the robot frame
        # print(vel)
        vel = np.matmul(self.J, vel)
        return vel

    def _callback(self, disp_msg, occ_msg):
        disp = image_to_numpy(disp_msg)
        occ = image_to_numpy(occ_msg) / 255.0

        cmd_vel = self._cal_control_action(disp, occ)
        
        # publish robot velocity
        vel = Twist()
        vel.linear.x = cmd_vel[0]
        vel.angular.z = cmd_vel[1]
        self.vel_pub.publish(vel)
        self.total_steps += 1
        print("FPS: %.2f" % ( self.total_steps / (time.time() - self.start_time)))


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("visual_servo")
    # Load target
    save_folder = os.path.join(pkg_path, "target")
    ref_disp = np.load(os.path.join(save_folder, "disp.npy"))
    ref_occ = np.load(os.path.join(save_folder, "occ.npy"))
    
    # Controller setting
    occ_aware = True
    gain = 30
    occ_thres = 0.7
    # camera settings
    fx = rospy.get_param("/visual_servo/focal_fx")
    fy = rospy.get_param("/visual_servo/focal_fy")
    img_w, img_h = rospy.get_param("/visual_servo/cropped_img_sz")
    cu, cv = rospy.get_param("/visual_servo/cropped_prinicipal_pt")
    baseline = rospy.get_param("/visual_servo/baseline")

    rospy.init_node("controller", anonymous=True)
    controller = DispController(
        ref_disp, ref_occ, gain, occ_thres, baseline, fx, fy, img_w, img_h, cu, cv, occ_aware)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
