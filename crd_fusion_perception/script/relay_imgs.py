#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from crd_fusion_perception.msg import StereoImgs
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ImageRelay:
    def __init__(self):
        self.left_img_sub = Subscriber("/camera/infra1/image_rect_raw", Image)
        self.right_img_sub = Subscriber("/camera/infra2/image_rect_raw", Image)
        self.raw_disp_sub = Subscriber("/camera/depth/image_rect_raw", Image)
        self.sub_synch = ApproximateTimeSynchronizer(
           [self.left_img_sub, self.right_img_sub, self.raw_disp_sub], queue_size=1, slop=0.05)
        self.sub_synch.registerCallback(self._callback)
        self.imgs_pub = rospy.Publisher("/raw_imgs", StereoImgs, queue_size=1)
    
    def _callback(self, left, right, disp):
        stereo_imgs = StereoImgs()
        stereo_imgs.left = left
        stereo_imgs.right = right
        stereo_imgs.raw_disp = disp
        self.imgs_pub.publish(stereo_imgs)


if __name__ == "__main__":
    rospy.init_node("img_relay", anonymous=True)
    img_relay = ImageRelay()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
