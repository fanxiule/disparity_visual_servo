#!/usr/bin/env python

import os
import time
import rospy
import rospkg
import numpy as np
import scipy.ndimage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from depth_visual_servo_common.image_np_conversion import image_to_numpy


# p^b_o = [0.045, 0.018, 0.104]
# Moving axis rotation: xb>-90, yb'>+90
# R^b_o = [0, 0, 1; -1, 0, 0; 0, -1, 0]

class DispController:
    def __init__(self, ref_disp, gain, baseline, fx, fy, img_w, img_h, orig_w, orig_h, cu, cv):
        """
        Controller for disparity based visual servoing

        :param ref_disp: reference disparity map
        :param gain: controller gain
        :param baseline: camera baseline
        :param fx: focal length in x
        :param fy: focal length in y
        :param img_w: image width
        :param img_h: image height
        :param cu: principal point in x
        :param cv: principal point in y
        """
        self.gain = gain
        self.fx = fx
        self.fy = fy
        self.baseline = baseline
        self.crop_x1 = (orig_w - img_w) // 2
        self.crop_y1 = (orig_h - img_h) // 2
        self.crop_x2 = self.crop_x1 + img_w
        self.crop_y2 = self.crop_y1 + img_h

        # flatten ref_disp
        self.upper_bound = 64
        self.ref_disp = ref_disp.flatten()
        ref_mask1 = self.ref_disp > 0
        ref_mask2 = self.ref_disp < self.upper_bound
        self.ref_mask = ref_mask1 * ref_mask2
        
        # create pixel grids
        u = np.arange(1, img_w + 1)
        v = np.arange(1, img_h + 1)
        u, v = np.meshgrid(u, v)
        self.x = u - cu
        self.y = v - cv

        # subscribers and publisher
        self.disp_sub = rospy.Subscriber("/raw_disp", Image, self._callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # a different strategy for camera-wheel velocity transformation
        self.J = np.array([[0, -0.045], [1, -0.018], [0, -1]])
        self.J = np.linalg.pinv(self.J)

        self.start_time = None
        self.total_steps = 0

    def _cal_control_action(self, disp, mask):
        """
        Calculate the control action

        :param disp: disparity map
        :param occ: validity mask for disparity
        :return: calculated velocity
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
        mask_control = mask.flatten()

        # select valid pixels in both the current view and the target view
        mask_control = mask_control * self.ref_mask

        # build the final interaction matrix
        L_d = np.stack((L_dv_x, L_dv_z, L_dw_y), axis=-1)

        # calculate the velocity
        L_d_inv = np.linalg.pinv(np.expand_dims(mask_control, -1) * L_d)
        # velocity in the camera frame
        vel = - self.gain * np.matmul(L_d_inv, mask_control * (disp_control - self.ref_disp))
        # velocity in the robot frame
        vel = np.matmul(self.J, vel)
        return vel

    def _callback(self, disp_msg):
        if self.start_time is None:
            self.start_time = time.time()

        disp = image_to_numpy(disp_msg)
        disp = disp[self.crop_y1:self.crop_y2,
                    self.crop_x1:self.crop_x2]

        mask1 = disp > 0
        mask2 = disp < self.upper_bound
        mask = mask1 * mask2

        cmd_vel = self._cal_control_action(disp, mask)

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
    # Load target
    save_folder = os.path.join(pkg_path, "target")
    ref_disp = np.load(os.path.join(save_folder, "raw_disp.npy"))

    # Controller setting
    gain = 3000
    # camera settings
    fx = rospy.get_param("/visual_servo/focal_fx")
    fy = rospy.get_param("/visual_servo/focal_fy")
    img_w, img_h = rospy.get_param("/visual_servo/cropped_img_sz")
    orig_w, orig_h = rospy.get_param("/visual_servo/orig_img_sz")
    cu, cv = rospy.get_param("/visual_servo/cropped_prinicipal_pt")
    baseline = rospy.get_param("/visual_servo/baseline")

    rospy.init_node("raw_disp_controller", anonymous=True)
    controller = DispController(
        ref_disp, gain, baseline, fx, fy, img_w, img_h, orig_w, orig_h, cu, cv)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
