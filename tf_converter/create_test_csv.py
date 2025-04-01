#!/usr/bin/env python
import numpy as np
import cv2
import copy
import rospy
import tf
import csv
import keyboard
from collections import deque
from cv_bridge import CvBridge
from common import TFcfg
from TF_converter import TFcvt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4*4,3), np.float32)
objp[:,:2] = np.mgrid[0:4,0:4].T.reshape(-1,2)

TF_cfg = TFcfg()
TF_cvt = TFcvt()
bridge = CvBridge()
depth_stack = deque([])
rgb_stack= deque([])
def append_rgb(msg):
    rgb_stack.append(bridge.imgmsg_to_cv2(msg))
    if len(rgb_stack) == 100:
        rgb_stack.popleft()   
    return
def append_depth(msg):
    depth_stack.append(bridge.imgmsg_to_cv2(msg))
    if len(depth_stack) == 100:
        depth_stack.popleft()

    return
def UV2RPoint(u,v,trans, rot):
    TF_cfg.img = depth_stack.pop()
    h_e_array = TF_cfg.h_e_array
    rotmat = TF_cvt.rotMat_from_quat(rot[0], rot[1], rot[2], rot[3])
    current_p = TF_cfg.get_robot_pose_mat(rotmat, trans)
    TF_cfg.TF_matrix = np.dot(current_p, h_e_array)
    result = TF_cvt.uv2point(TF_cfg,u,v)
    return result[0], result[1], result[2]

    

def main():
    rospy.init_node("CSV_Test")
    TF_cfg.set_tf_listener()
    rospy.Subscriber(TF_cfg.img_msg_name,Image,append_rgb)
    rospy.Subscriber(TF_cfg.depth_img_msg_name,Image,append_depth)
    rate = rospy.Rate(30)
    csvfile = None
    print('please input csvfile name')
    filename = raw_input()
    filename += '.csv'
    filelength = 0
    #select csvfile io mode
    print('please select csvfile mode ([0]: overwrite, [1]: followwrite, [other]: terminate')
    ans = raw_input()
    if int(ans) == 0:
        csvfile = open(filename, 'w')
    elif int(ans) == 1:
        tmpfile = open(filename, 'r')
        rdr = csv.reader(tmpfile)
        filelength = sum(1 for row in rdr)
        csvfile = open(filename, 'a')
    else:
        return
    wr = csv.writer(csvfile)
    #system manage
    flag = True
    #user message
    text_flag = True
    while not rospy.is_shutdown():
        #ground truth
        try:
            (trans, rot) = TF_cfg.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #img
        if len(rgb_stack) == 0:
            continue
        if len(depth_stack) == 0:
            continue
        objpoints = []
        imgpoints = []
        bgr_image = rgb_stack.pop()
        gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_image, (4,4), None)
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray_image, corners, (5,5), (-1,-1), criteria)
            imgpoints.append(corners2)
            rgb_image = cv2.drawChessboardCorners(rgb_image, (4,4), corners2, ret)
            point_shape = corners2.shape
            if(text_flag == True): 
                print("enter 's' write current points")
                text_flag = False
            if keyboard.is_pressed('s') and flag:
                print('\n write %d' % (filelength+1))
                for i in range (0,point_shape[0]):
                    print("point[%d]: (%d,%d)"% (i+1,corners2[i][0][0],corners2[i][0][1] ))
                    x,y,z= UV2RPoint(int(corners2[i][0][0]), int(corners2[i][0][1]), trans, rot)
                    wr.writerow([filelength+(i+1), str(x[0]),str(y[0]),str(z[0])])
                flag = False
                print("enter 'c' continue, enter 'n' exit")
        if keyboard.is_pressed('c') and flag == False:
            print('\ninput mode')
            flag = True
            text_flag = True
        if keyboard.is_pressed('n'):
            break
            #print('corners_shape: ',corners2)
            #cv2.imshow('img',rgb_image)
            #cv2.waitKey(0)
        cv2.imshow('img',rgb_image)
        cv2.waitKey(1)
        rate.sleep()
    return
if __name__ == "__main__":
    main()
    