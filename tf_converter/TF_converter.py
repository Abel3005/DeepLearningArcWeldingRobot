import math
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

"""@package docstring

Mudule for TF convert

"""
class TFcvt:
    def __init__(self):
        return



    """Get 4*4 transform matrix from Quaternion (only rotation 3*3 area is changed)"""
    def rotMat_from_quat(self, x,y,z,w):
        r,p,y = self.euler_from_quaternion(x, y, z, w)
        return self.euler_to_rotMat(y,p,r)


    """Get Euler RPY from quaternion"""
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z # in radians

    """Get 4*4 transform matrix from Euler RPY (only rotation 3*3 area is changed)"""
    def euler_to_rotMat(self, yaw, pitch, roll):
        Rz_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [          0,            0, 1]])
        Ry_pitch = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [             0, 1,             0],
            [-np.sin(pitch), 0, np.cos(pitch)]])
        Rx_roll = np.array([
            [1,            0,             0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]])
        rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
        return rotMat

    """Convert coord from image plane to camera frame """
    def deprojection(self, x, y, depth):
        if depth==0:
            return None, None, None
        cor_x = float(depth)/1000
        scale = cor_x/1.88

        cor_y = float((x) - 640)*scale/-1000
        cor_z =  float((y)-360)*scale/-1000
        return cor_x, cor_y, cor_z


    """Transform xyz coord with 4*4 transform matrix  """
    def tfPoint(self, x, y, z, mat):
        vector4 =  np.array([[x],[y],[z],[1]])
        result = np.dot(mat, vector4) # base -> point(index1)
        return result


    """Publish point cloud data by TF config instance """
    def pc_pub(self, TF_cfg):
        PC = PointCloud()
        PC.header.frame_id='base_link'
        idx = 0
        for y in range(TF_cfg.img_height):
            for x in range(TF_cfg.img_width):
                idx +=1
                if idx % 100 != 0:
                    continue
                
                cor_x, cor_y, cor_z = self.deprojection(x, y, TF_cfg.img[y][x])
                if not cor_x:
                    continue
                result_1 = self.tfPoint(cor_x, cor_y, cor_z,TF_cfg.TF_matrix)
                point = Point32()
                point.x=(result_1[0])
                point.y=(result_1[1])
                point.z=(result_1[2])
                PC.points.append(point)
        TF_cfg.PC_pub.publish(PC)
        return
    
    """uv to robot base coordinates"""
    def uv2point(self, TF_cfg, u, v):
        cor_x, cor_y, cor_z = self.deprojection(u, v, TF_cfg.img[v][u])
        if not cor_x:
            return np.array(0,1,2,1)
        result = self.tfPoint(cor_x, cor_y, cor_z,TF_cfg.TF_matrix)
        return result
        