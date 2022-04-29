#!/usr/bin/env python

import rospy


def set_param():
    """
    Load common parameters for the visual servoing algorithm to the parameter server
    """
    rospy.init_node("common_param", anonymous=True)

    # Update these according to your settings
    # In simulation, orig_img_w and orig_img_h can be set in d435.gazebo.xacro
    # focal_fx and focal_fy can be found by running $ rostopic echo /camera/infra1/camera_info
    # baseline can be found by running $ rosrun tf tf_echo camera_left_ir_frame camera_right_ir_frame
    # d435 848 x 480
    """focal_fx = 426.077
    focal_fy = 426.077
    orig_img_w = 848
    orig_img_h = 480
    orig_cu = 422.366
    orig_cv = 241.651"""

    # d435 640 x 480
    """focal_fx = 385.881
    focal_fy = 385.881
    orig_img_w = 640
    orig_img_h = 480
    orig_cu = 318.520
    orig_cv = 241.496"""

    # d415 848 x 480
    """focal_fx = 608.263
    focal_fy = 608.263
    orig_img_w = 848
    orig_img_h = 480
    orig_cu = 418.423
    orig_cv = 245.974"""

    # d415 640 x 480
    focal_fx = 608.263
    focal_fy = 608.263
    orig_img_w = 640
    orig_img_h = 480
    orig_cu = 314.423
    orig_cv = 245.974

    cropped_img_w = 320
    cropped_img_h = 240
    cropped_cu = orig_cu - (orig_img_w - cropped_img_w) / 2.0
    cropped_cv = orig_cv - (orig_img_h - cropped_img_h) / 2.0
    
    # d435
    # baseline = 50.146 / 1000
    # d415
    baseline = 55.003 / 1000

    rospy.set_param("/visual_servo/focal_fx", focal_fx)
    rospy.set_param("/visual_servo/focal_fy", focal_fy)
    rospy.set_param("/visual_servo/orig_img_sz", [orig_img_w, orig_img_h])
    rospy.set_param("/visual_servo/orig_principal_pt", [orig_cu, orig_cv])
    rospy.set_param("/visual_servo/cropped_img_sz",
                    [cropped_img_w, cropped_img_h])
    rospy.set_param("/visual_servo/cropped_prinicipal_pt",
                    [cropped_cu, cropped_cv])
    rospy.set_param("/visual_servo/baseline", baseline)

    print("-----------------------------------------------")
    print("Loaded common parameters to param server")
    print("Focal length: %.2f, %.2f" % (focal_fx, focal_fx))
    print("Original image size: %d x %d" % (orig_img_w, orig_img_h))
    print("Original principal point: (%.2f, %.2f)" % (orig_cu, orig_cv))
    print("Cropped image size: %d x %d" % (cropped_img_w, cropped_img_h))
    print("Cropped principal point: (%.2f, %.2f)" % (cropped_cu, cropped_cv))
    print("Baseline: %.4f m" % baseline)
    print("-----------------------------------------------")


if __name__ == '__main__':
    try:
        set_param()
    except rospy.ROSInterruptException:
        pass
