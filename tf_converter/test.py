#!/usr/bin/env python
import rospy

import numpy as np

from sensor_msgs.msg import Image

import tf
from TF_converter import TFcvt
from common import TFcfg


TF_cfg = TFcfg()
TF_cvt = TFcvt()

def img2pc_pub(msg):

    try:
        (trans, rot) = TF_cfg.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    #------------- GET TF matrix
    h_e_array = TF_cfg.h_e_array
    rotmat = TF_cvt.rotMat_from_quat(rot[0], rot[1], rot[2], rot[3])
    current_p = TF_cfg.get_robot_pose_mat(rotmat, trans)
    TF_cfg.TF_matrix = np.dot(current_p, h_e_array)


    #------------- Cvt rosMsg to img
    TF_cfg.set_img_from_rosMsg(msg)

    #------------- Publish pointcloud
    TF_cvt.pc_pub(TF_cfg)
    return

def main():
    rospy.init_node("TF_test")

    TF_cfg.set_tf_listener()
    rospy.Subscriber(TF_cfg.depth_img_msg_name,Image, img2pc_pub, queue_size=1)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
    return
if __name__=="__main__":
    main()