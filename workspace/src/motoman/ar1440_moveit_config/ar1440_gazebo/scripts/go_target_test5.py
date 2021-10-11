#!/usr/bin/env python
#-*- coding: utf-8 -*-
## 파이썬 MoveIt 인터페이스를 사용하려면 `moveit_commander`_ 네임스페이스를 import 합니다.
## 이 네임스페이스는 `MoveGroupCommander`, `PlanningSceneInterface`, `RobotCommander` 클래스를 제공합니다.
## 또한 `rospy` 와 메시지들을 import 합니다:
##

import sys
import os
import copy
import time
import numpy as np
from moveit_commander import move_group
import rospy
import serial # HANA ros_serial 패키지 의존성
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
import tf
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
import math

from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
depth_img_msg_name="/camera/aligned_depth_to_color/image_raw"

def rotMat_from_quat(x,y,z,w):
    r,p,y = euler_from_quaternion(x, y, z, w)
    return euler_to_rotMat(y,p,r)
def euler_from_quaternion(x, y, z, w):
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
def euler_to_rotMat(yaw, pitch, roll):
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

def message_callback_tf( message):
    for msg in message.transforms:
      if msg.child_frame_id == 'torch_ee':
        print("== torch_ee ==")
        print(msg.transform.translation.x)
        print(msg.transform.translation.y)
        print(msg.transform.translation.z)

def all_close(goal, actual, tolerance):
  """
  수치값들의 목록이 실제 값들과 공차(tolerance) 범위 안에 있는지 검사하는 메소드
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def receive_torch_coord_callback(data):
  global torch_poses
  torch_poses = data

  return

yolo_poses = Path()

def receive_3d_coord_callback(data):
  global yolo_poses
  yolo_poses = data

  #if(yolo_poses.poses != None):
  global get_pose_bool
  get_pose_bool = True
  #print("init yolo data")

  return

class MoveArm(object):
  """MoveArm"""
  def __init__(self):
    super(MoveArm, self).__init__()
    ## 먼저 `moveit_commander` 와 `rospy` 노드를 초기화합니다:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)


    ## `RobotCommander` 객체를 생성합니다.
    ## 이 객체는 로봇의 기구학 모델과 현재의 관절 상태와 같은 정보를 제공합니다.
    robot = moveit_commander.RobotCommander()

    ## `PlanningSceneInterface` 객체를 생성합니다.
    ## 이 객체는 로봇의 주변 환경에 대한 정보를 획득, 설정, 갱신하는 인터페이스를 제공합니다.
    scene = moveit_commander.PlanningSceneInterface()

    ## `MoveGroupCommander` 객체를 생성합니다.
    ## 이 객체는 플래닝 그룹(관절 그룹)에 대한 인터페이스를 제공합니다.
    ## 여기서는 ar1440 로봇의 관절 그룹의 이름을 "manipulator"로 설정합니다.
    ## 이 인터페이스는 모션을 계획하고 실행하는 데 사용됩니다:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Rviz에서 궤적을 표시하는 데 사용되는 `DisplayTrajectory` ROS 퍼블리셔를 생성합니다.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    ## 기본 정보 획득
    ## ^^^^^^^^^^^^^
    # 로봇에 대한 참조 프레임의 이름을 출력합니다
    planning_frame = move_group.get_planning_frame()
    #print("============ Planning frame: %s" % planning_frame)

    # 그룹에 대한 말단장치 링크의 이름을 출력합니다
    eef_link = move_group.get_end_effector_link()
    #print("============ End effector link: %s" % eef_link)

    # 로봇의 모든 그룹 목록을 출력합니다
    group_names = robot.get_group_names()
    #print("============ Available Planning Groups:", robot.get_group_names())
    
    # 로봇의 현재 상태를 출력합니다
    print("============ Printing robot state")
    print(robot.get_current_state())
    #print("")

    # 추가 변수들
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.listener = tf.TransformListener()
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    move_group = self.move_group

    joint_goal = [0, 0, 0, 0, 0, 0]
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  # 첫 번째 용접 포인트로 이동하는 함수
  def go_to_pose_goal(self):
    move_group = self.move_group

    pose_goal = geometry_msgs.msg.Pose()

    current_pose = self.move_group.get_current_pose().pose 

    q = quaternion_from_euler(-3.1415927, -1.395, 0.0)  # end-effector의 회전변환 (y축 기준 79도)
    
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2] 
    pose_goal.orientation.w = q[3] 


    try:
        (trans,rot) = self.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("TF ERROR----------------------------------------")
        return
    now_trans = trans[:]
    now_rot = rot[:]
    rotmat = rotMat_from_quat(now_rot[0], now_rot[1], now_rot[2], now_rot[3])

#    h_e_array = np.array([  [0.9848077,  0.0000000, -0.1736482,  0.112 ],
#                            [0.0000000,  1.0000000,  0.0000000, -0.025],
#                            [0.1736482,  0.0000000,  0.9848077,  0.153 ],
#                            [0         ,  0        ,  0        , 1]])
    h_e_array = np.array([  [0.9455186,  0.0000000, 0.3255682,  0.112 ],
                            [0.0000000,  1.0000000,  0.0000000, -0.025],
                            [-0.3255682,  0.0000000,  0.9455186,  0.153 ],
                            [0         ,  0        ,  0        , 1]])

    current_p = np.array([[ rotmat[0][0],      rotmat[0][1],   rotmat[0][2], now_trans[0]-0.402794],
                          [ rotmat[1][0],      rotmat[1][1],   rotmat[1][2], now_trans[1]-0.002158351],
                          [ rotmat[2][0],      rotmat[2][1],   rotmat[2][2], now_trans[2]+0.156928],
                          [ 0,          0,          0,         1                      ]])



    # 수정해야 할 value (camera -> welding point transformation matrix) 
#    e_p_array1 = np.array([[Save_yolo_poses.poses[0].pose.position.x], [Save_yolo_poses.poses[0].pose.position.y], [Save_yolo_poses.poses[0].pose.position.z],[1]])
    e_p_array1 = np.array([[0  , 0,   0, Save_yolo_poses.poses[0].pose.position.x],
                           [0 , 0,  0, Save_yolo_poses.poses[0].pose.position.y],
                           [0  , 0,  0, Save_yolo_poses.poses[0].pose.position.z],
                           [0, 0, 0, 1                                                                  ]])
    e_p_array2 = np.array([[0  , 0,   0, Save_yolo_poses.poses[1].pose.position.x],
                           [0 , 0,  0, Save_yolo_poses.poses[1].pose.position.y],
                           [0  , 0,  0, Save_yolo_poses.poses[1].pose.position.z],
                           [0, 0, 0, 1   ]])
    e_p_array3 = np.array([[0  , 0,   0, Save_yolo_poses.poses[2].pose.position.x],
                           [0 , 0,  0, Save_yolo_poses.poses[2].pose.position.y],
                           [0  , 0,  0, Save_yolo_poses.poses[2].pose.position.z],
                           [0, 0, 0, 1   ]])
#    e_p_array1 = np.array([[0.0042092690  , -0.9994275,   0.0335694900, 0.3],
#                           [-0.5539359634 , -0.0302790,  -0.8320084543, 0.3],
#                           [0.8325486466  , -0.0150931,  -0.5537463389, 0.3],
#                           [0, 0, 0, 1                                                                  ]])

    #print("e_parray1 : " , Save_yolo_poses.poses[0].pose.position)

    # Before H/E Calibration
    # e_p_array1 = np.array([[0.0321040 , -0.9793274, -0.1997180, Save_yolo_poses.poses[0].pose.position.x],
    #                        [0.9994467 ,  0.0297166,  0.0149407, Save_yolo_poses.poses[0].pose.position.y],
    #                        [-0.0086969, -0.2000871,  0.9797395, Save_yolo_poses.poses[0].pose.position.z],
    #                        [0, 0, 0, 1                                     ]])

#    e_p_array2 = np.array([[0.0042092690  , -0.9994275,   0.0335694900, Save_yolo_poses.poses[1].pose.position.z],
#                           [-0.5539359634 , -0.0302790,  -0.8320084543, Save_yolo_poses.poses[1].pose.position.y],
#                           [0.8325486466  , -0.0150931,  -0.5537463389, Save_yolo_poses.poses[1].pose.position.x],
#                           [0, 0, 0, 1                                       ]])


    result_ = np.dot(current_p, h_e_array) # base -> camera

    result_1 = np.dot(result_, e_p_array1) # base -> point(index1)
    result_2 = np.dot(result_, e_p_array2) # base -> point(index2)
    result_3 = np.dot(result_, e_p_array3) # base -> point(index3)

    # 1번 인덱스 용접 포인트의 위치(이동되는 좌표)    
    print(type( result_1[0]))              #Margin
    pose_goal.position.x =  result_1[0][3] + 0.1             #- 0.402794
    pose_goal.position.y =  result_1[1][3] + 0.035             #- 0.002158351 
    pose_goal.position.z =  result_1[2][3] - 0.06            #+0.1  #- 0.450 #+ 0.156928  #+450 : 팬던트 상에서는 450mm가 더해져서 나오기때문에 코드상에서는 더해줘야함.


    print("== base to end-effector ==")
    print(current_pose.position.x)
    print(current_pose.position.y)
    print(current_pose.position.z)

    print("== base to camera ==")
    print(result_[0])
    print(result_[1])
    print(result_[2])

    print("== camera to welding point ==")
    print(Save_yolo_poses.poses[0].pose.position.x)
    print(Save_yolo_poses.poses[0].pose.position.y)
    print(Save_yolo_poses.poses[0].pose.position.z)

    print("== base to welding point ==")
    print(pose_goal.position.x)
    print(pose_goal.position.y) #pose_goal
    print(pose_goal.position.z) #pose_goal

    raw_input()

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    # time.sleep(1)

    # pose_goal.orientation.x = q[0] #0.97 #0.0
    # pose_goal.orientation.y = q[1] #0.0 #0.0
    # pose_goal.orientation.z = q[2] #0.23 #0.0
    # pose_goal.orientation.w = q[3] #0.0 #1.0

    # pose_goal.position.x =  0.495058  
    # pose_goal.position.y =  0.004167  
    # pose_goal.position.z =  0.683743  

    # move_group.set_pose_target(pose_goal)

    # # 자세(pose) 목표로 로봇 관절 그룹을 움직이도록 go 명령을 호출한다.
    # plan = move_group.go(wait=True)
    # # 남은 로봇 움직임이 없다는 것을 확인하기 위해 ``stop()``을 호출한다
    # move_group.stop()
    # # 자세로 플래닝한 후에 목표 지점들을 지운다
    # # 주의: 관절 목표에 대한 clear_joint_value_targets() 함수는 없다
    # move_group.clear_pose_targets()

    # 최종 로봇 위치 검사
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
  # 홈 포지션으로 이동하는 함수
  def go_to_home_pose(self):

    move_group = self.move_group

    pose_goal = geometry_msgs.msg.Pose()

    q = quaternion_from_euler(-3.1415927, -1.1853982, 0.0)  # end-effector의 회전변환 (y축 기준 67도)
    pose_goal.orientation.x = q[0] #0.97 #0.0
    pose_goal.orientation.y = q[1] #0.0 #0.0
    pose_goal.orientation.z = q[2] #0.23 #0.0
    pose_goal.orientation.w = q[3] #0.0 #1.0

    # 홈 포지션에서의 end-effector의 위치(base 기준)
    pose_goal.position.x =  0.388350 #0.268350 #(m)
    pose_goal.position.y =  0.0      #0.019599 #(m)
    pose_goal.position.z =  0.801373 #0.861373 #(m) 

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # 최종 로봇 위치 검사
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, pose, scale=1):
    move_group = self.move_group

    waypoints = []

    wpose = move_group.get_current_pose().pose
    diffx = pose.position.x - wpose.position.x
    diffy = pose.position.y - wpose.position.y
    diffz = pose.position.z - wpose.position.z

    # 첫째경유지점: 위쪽 방향(z)으로 조금 이동
    wpose.position.z += scale * 0.10 + diffz
    waypoints.append(copy.deepcopy(wpose))

    # 둘째경유지점:  앞/뒤 방향(x) 및 옆 방향(y)로 이동
    wpose.position.x += scale * diffx
    wpose.position.y += scale * diffy

    waypoints.append(copy.deepcopy(wpose))

    # 셋째경유지점: 아래쪽 방향(-z)으로 조금 이동
    wpose.position.z -= scale * 0.10

    waypoints.append(copy.deepcopy(wpose))

    # 카르테시안 경로를 계산
    # eef_step를 0.01로 지정하여 1cm 해상도로 interpolate 되도록 한다
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction

  def plan_execute_pose_goal(self, pose, scale=1):

    move_group = self.move_group

    waypoints = []

    wpose = move_group.get_current_pose().pose
    diffx = pose.position.x - wpose.position.x
    diffy = pose.position.y - wpose.position.y
    diffz = pose.position.z - wpose.position.z

    # 첫째경유지점: 위쪽 방향(z)으로 조금 이동한다
    wpose.position.z += scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    # 둘째경유지점:  앞/뒤 방향(x) 및 옆 방향(y)로 이동한다
    wpose.position.x += scale * diffx
    wpose.position.y += scale * diffy
    wpose.position.z += diffz
    waypoints.append(copy.deepcopy(wpose))

    # 셋째경유지점: 아래쪽 방향(-z)으로 조금 이동한다
    wpose.position.z -= scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    for waypoint in waypoints:
        move_group.set_pose_target(waypoint)

        plan = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        # 최종 로봇 위치 검사
        current_pose = self.move_group.get_current_pose().pose
        if not all_close(waypoint, current_pose, 0.01):
            return False

    return True

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish
    display_trajectory_publisher.publish(display_trajectory)

  def execute_plan(self, plan):
    move_group = self.move_group

    move_group.execute(plan, wait=True)

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():

      # 인자로 받은 박스가 부착된(attached) 객체인지 검사
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # 박스가 씬 안에 있는지 검사
      # 박스를 부착하면 인식(known) 객체 목록에서 제거된다는 점에 주의
      is_known = box_name in scene.get_known_object_names()

      # 기대한 상태에 있는지 검사
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # 다른 스레드들이 실행되도록 잠시 슬립한다
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # while 루프에서 반환하지 않고 빠져나오면 타임아웃된 것으로 반환
    return False

  def add_box_object(self, name, dimensions, pose):
      p = geometry_msgs.msg.PoseStamped()
      p.header.frame_id = self.move_group.get_planning_frame()
      p.header.stamp = rospy.Time.now()
      p.pose.position.x = pose[0]
      p.pose.position.y = pose[1]
      p.pose.position.z = pose[2]
      p.pose.orientation.x = pose[3]
      p.pose.orientation.y = pose[4]
      p.pose.orientation.z = pose[5]
      p.pose.orientation.w = pose[6]

      self.scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))

  def add_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    box1_pose = [1.0, 0.4, 0.35, 0, 0, 0, 1]
    box1_dimensions = [1.0, 0.1, 0.7]
    self.add_box_object("box1", box1_dimensions, box1_pose)
    self.wait_for_state_update("box1", box_is_known=True, timeout=timeout)

    box2_pose = [1.0, -0.4, 0.35, 0, 0, 0, 1]
    box2_dimensions = [1.0, 0.1, 0.7]
    self.add_box_object("box2", box2_dimensions, box2_pose)
    self.wait_for_state_update("box2", box_is_known=True, timeout=timeout)

    box3_pose = [1.0, 0.0, 0.05, 0, 0, 0, 1]
    box3_dimensions = [1.0, 0.7, 0.1]
    self.add_box_object("box3", box3_dimensions, box3_pose)
    self.wait_for_state_update("box3", box_is_known=True, timeout=timeout)

  def remove_box(self, timeout=4):
    scene = self.scene

    scene.remove_world_object("box1")
    scene.remove_world_object("box2")
    scene.remove_world_object("box3")

    self.wait_for_state_update("box1", box_is_attached=False, box_is_known=False, timeout=timeout)
    self.wait_for_state_update("box2", box_is_attached=False, box_is_known=False, timeout=timeout)
    self.wait_for_state_update("box3", box_is_attached=False, box_is_known=False, timeout=timeout)

get_pose_bool = False

def main():

  global yolo_poses

  group_name = "manipulator"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  try:
    
    #os.system("rosservice call /robot_enable") # 로봇 서보전원 ON
    
    global get_pose_bool


    # serial port 통신
    port = rospy.get_param('~port', "/dev/ttyUSB0")
    
    try:
      ser = serial.Serial(port=port, baudrate=115200, timeout=1)
    except serial.serialutil.SerialException:
      sys.exit(0)

    tutorial = MoveArm()

    #======================================= Robot control start =======================================#

    # 홈 포지션으로 이동
    tutorial.go_to_home_pose() 

    raw_input()

    # 카메라 좌표 받아오기
    
    yolo_3d_sub = rospy.Subscriber("/darknet_3d/sorted_poses",Path,receive_3d_coord_callback)
    global Save_yolo_poses
    Save_yolo_poses = Path()

    while not rospy.is_shutdown(): # DATA_RECEIVED 
      global yolo_poses
      print("Waiting Yolo Pose")
      rospy.sleep(0.1)
      print("Detection Yolo Poses " + str(len(yolo_poses.poses)))

      print(get_pose_bool)
      if(get_pose_bool == True and len(yolo_poses.poses) == 3):
        print("init yolo poses")
        print(yolo_poses.poses)
        Save_yolo_poses = yolo_poses
        break
    
    torch_poses = TFMessage()
    torch_coord_sub = rospy.Subscriber("/tf",TFMessage,receive_torch_coord_callback) 
    print("torch_poses")

    print("execute a movement using a 1st pose goal")
    
    # 첫 번째 용접 포인트로 이동
    tutorial.go_to_pose_goal()

    raw_input()

    # # 서보전원 off
    # os.system("rosservice call /robot_disable")

    # # welding JOB start (1st point) , 'D' 메시지를 전달하여 JOB 실행
    # times = 1
    # for i in range(times):
    #   ser.write("D")
    #   rospy.sleep(0.001)
    # time.sleep(4) # 용접시간만큼 딜레이

    print("plan and display a Cartesian path with 2nd goal")



    # 서보 전원 다시 ON
    #os.system("rosservice call /robot_enable")
    #tutorial.go_to_home_pose()
    raw_input()
    # 두 번째 용접 포인트로 이동 
    q = quaternion_from_euler(-3.1415927, -1.395, 0.0)
    goal2 = geometry_msgs.msg.Pose()
    goal2.orientation.x = q[0]
    goal2.orientation.y = q[1]
    goal2.orientation.z = q[2]
    goal2.orientation.w = q[3]

    h_e_array = np.array([[0.0321040 , -0.9793274, -0.1997180,  0.09274130216446597 ],
                          [0.9994467 ,  0.0297166,  0.0149407, -0.025195150523802758],
                          [-0.0086969, -0.2000871,  0.9797395,  0.1235782699402816  ],
                          [0         ,  0        ,  0        ,  1                   ]])

    current_p = np.array([[ 0.3753989,  0.0007699,  0.9268630, 0.388379219559    ],
                          [ 0.0009996, -0.9999994,  0.0004258, -1.65217500806e-06],
                          [ 0.9268628,  0.0007667, -0.3753995, 0.801371910358    ],
                          [ 0,          0,          0,         1                 ]]) 

    e_p_array1 = np.array([[1, 0, 0, Save_yolo_poses.poses[0].pose.position.y],
                           [0, 1, 0, Save_yolo_poses.poses[0].pose.position.z],
                           [0, 0, 1, Save_yolo_poses.poses[0].pose.position.x],
                           [0, 0, 0, 1                                       ]])

    e_p_array2 = np.array([[1, 0, 0, Save_yolo_poses.poses[1].pose.position.y],
                           [0, 1, 0, Save_yolo_poses.poses[1].pose.position.z],
                           [0, 0, 1, Save_yolo_poses.poses[1].pose.position.x],
                           [0, 0, 0, 1                                       ]])

    # e_p_array3 = np.array([[1, 0, 0, Save_yolo_poses.poses[2].pose.position.y],
    #                        [0, 1, 0, Save_yolo_poses.poses[2].pose.position.z],
    #                        [0, 0, 1, Save_yolo_poses.poses[2].pose.position.x],
    #                        [0, 0, 0, 1                                       ]])

    e_p_array3 = np.array([[1, 0, 0, Save_yolo_poses.poses[1].pose.position.y],
                           [0, 1, 0, Save_yolo_poses.poses[1].pose.position.z],
                           [0, 0, 1, Save_yolo_poses.poses[1].pose.position.x],
                           [0, 0, 0, 1                                       ]])


    result_ = np.dot(current_p, h_e_array)
    result_1 = np.dot(result_, e_p_array1)
    result_2 = np.dot(result_, e_p_array2)
    result_3 = np.dot(result_, e_p_array3)

    # 이동할 좌표 (2번 인덱스)
    goal2.position.x = result_2[2][3]
    goal2.position.y = result_2[1][3]
    goal2.position.z = result_2[0][3] -0.400

    print(goal2.position.x)
    print(goal2.position.y)
    print(goal2.position.z)

    result = tutorial.plan_execute_pose_goal(goal2)
    if not result:
        print("------ Goal not reached!!! ------")

    raw_input()

    # 서보 전원 off
    # os.system("rosservice call /robot_disable")

    ## welding JOB start (2nd point)

    # 'E' 메시지를 전달하여 JOB(용접) 실행
    # for i in range(times):
    #   ser.write("E")
    #   rospy.sleep(0.001)

    # 용접작업 중 sleep (용접소요 time: 3.5sec)
    #time.sleep(4)

    print("plan and execute a path with 3rd goal")
    
    # 서보 전원 다시 on
    #os.system("rosservice call /robot_enable")
    time.sleep(1)

    # 세 번째 용접 포인트로 이동
    # current_pose = move_group.get_current_pose().pose # base to eef
    goal3 = geometry_msgs.msg.Pose()
    goal3.orientation.x = q[0]
    goal3.orientation.y = q[1]
    goal3.orientation.z = q[2]
    goal3.orientation.w = q[3]

    goal3.position.x = result_3[2][3] 
    goal3.position.y = result_3[1][3] 
    goal3.position.z = result_3[0][3] - 0.400
    result = tutorial.plan_execute_pose_goal(goal3)

    if not result:
        print("------ Goal not reached!!! ------")

    raw_input()

    # 서보 전원 off
    # os.system("rosservice call /robot_disable")

    ## welding JOB start (3rd point)

    # 'D' 메시지를 전달하여 JOB(용접) 실행
    # for i in range(times):
    #   ser.write("D")
    #   rospy.sleep(0.001)

############################################# 3point welding demo end ####################################################################

    # print("plan and execute a path with 4th goal")
    # time.sleep(1)
    # current_pose = move_group.get_current_pose().pose # base to eef
    # goal4 = geometry_msgs.msg.Pose()
    # goal4.orientation.x = q[0]
    # goal4.orientation.y = q[1]
    # goal4.orientation.z = q[2]
    # goal4.orientation.w = q[3]
    # goal4.position.x = 0.607815 #0.836625
    # goal4.position.y = 0.011313
    # goal4.position.z = 0.688342 #0.692759
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(goal4)

    # tutorial.execute_plan(cartesian_plan)
    
    # # rosservice call /robot_disable

    # # welding JOB start (4th point)
    # # for i in range(times):
    # #   ser.write("E")
    # #   rospy.sleep(0.001)

    # # rosservice call /robot_enable

    # time.sleep(1)

    # 모든 용접 완료 후, 홈 포지션으로 돌아옴
    tutorial.go_to_home_pose()

    print("Python tutorial demo complete.")

    # 'F' 메시지를 송신하여 ethernet disconnect(motocom32 통신 종료)
    for i in range(times):
      ser.write("F")
      rospy.sleep(0.001)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
  

if __name__ == '__main__':
  main()
