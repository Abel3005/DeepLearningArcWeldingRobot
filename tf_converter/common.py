import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud
import tf

"""@package docstring

Mudule for TF configuration

"""
class TFcfg:
    def __init__(self):
        self.flag = True
        self.img = None
        self.img_width = 1280
        self.img_height = 720
        self.depth_img_msg_name = "/camera/aligned_depth_to_color/image_raw"
        self.img_msg_name = "/camera/color/image_raw"
        self.listener = None
        self.TFmatrix = None
        #XYZ 080
        self.h_e_array = np.array([     [0.9902681,   0.0000000,  0.1391731,  0.15 ],
                                        [0.0000000,   1.0000000,  0.0000000, -0.025],
                                        [-0.1391731,  0.0000000,  0.9902681,  0.13 ],
                                        [0         ,  0        ,  0        ,    1  ]  ])
        self.current_p = np.array([     [1,0,0,0],
                                        [0,1,0,0],
                                        [0,0,1,0],
                                        [0,0,0,1]])


        self.PC_pub = rospy.Publisher('/temp_c', PointCloud,queue_size=1)
        return

    """set tf data from rotation, translation matrix"""
    def set_current_robot_pose(self, rotmat, transmat):
        self.current_p = np.array([ [ rotmat[0][0],      rotmat[0][1],   rotmat[0][2], transmat[0]],
                                    [ rotmat[1][0],      rotmat[1][1],   rotmat[1][2], transmat[1]],
                                    [ rotmat[2][0],      rotmat[2][1],   rotmat[2][2], transmat[2]],
                                    [ 0,          0,          0,         1                      ]])
        return

    """set liestener - AFTER rospy.init"""
    def set_tf_listener(self):
        self.listener = tf.TransformListener()
        return

    def get_robot_pose_mat(self, rotmat, transmat):
        mat = np.array([            [ rotmat[0][0],      rotmat[0][1],   rotmat[0][2], transmat[0]],
                                    [ rotmat[1][0],      rotmat[1][1],   rotmat[1][2], transmat[1]],
                                    [ rotmat[2][0],      rotmat[2][1],   rotmat[2][2], transmat[2]],
                                    [ 0,          0,          0,         1                      ]])        
        return mat

    """Convert rosMsg to img"""
    def set_img_from_rosMsg(self, msg):
        bridge = CvBridge()
        self.img = bridge.imgmsg_to_cv2(msg)