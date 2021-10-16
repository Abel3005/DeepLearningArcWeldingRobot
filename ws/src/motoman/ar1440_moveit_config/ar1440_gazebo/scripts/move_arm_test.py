#!/usr/bin/env python
#-*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## 파이썬 MoveIt 인터페이스를 사용하려면 `moveit_commander`_ 네임스페이스를 import 합니다.
## 이 네임스페이스는 `MoveGroupCommander`, `PlanningSceneInterface`, `RobotCommander` 클래스를 제공합니다.
## 또한 `rospy` 와 메시지들을 import 합니다:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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
    print("============ Planning frame: %s" % planning_frame)

    # 그룹에 대한 말단장치 링크의 이름을 출력합니다
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # 로봇의 모든 그룹 목록을 출력합니다
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # 로봇의 현재 상태를 출력합니다
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    # 추가 변수들
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # 클래스 변수를 지역 변수로 복사
    move_group = self.move_group

    ## 관절(Joint) 목표(Goal)로 플래닝
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## 먼저 AR1440 로봇의 관절 위치를 테스트하기 위해 기본 관절 제로(zero)
    ## 상태에서 2번째 관절을 움직여 로봇의 팔을 조금 내린다.
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = pi/4
    joint_goal[2] = 0
    joint_goal[3] = pi/2
    joint_goal[4] = 0
    joint_goal[5] = 0

    # 관절(joint) 또는 자세(pose) 목표로 로봇 관절 그룹을
    # 실제로 움직이기 위해 go 명령을 호출한다.
    move_group.go(joint_goal, wait=True)

    # 남은 로봇 움직임이 없다는 것을 확인하기 위해 ``stop()``을 호출한다
    move_group.stop()

    # 최종 로봇 위치 검사
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    # 클래스 변수를 지역 변수로 복사
    move_group = self.move_group

    ## 자세(Pose) 목표(Goal)로 플래닝
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 말단장치(end-effector)에 대한 원하는 자세까지 모션을 계획한다
    pose_goal = geometry_msgs.msg.Pose()
    ## 자세(Pose)는 위치(position)와 방향(orientation) 벡터로 구성된다
    ## 위치(position)은 x, y, z 의 3차원 벡터이며
    ## 방향(orientation)은 x, y, z, w 의 쿼터니언(quaternion) 정규화 벡터이다
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 1.25 #0.4
    pose_goal.position.y = 0.0 #0.1
    pose_goal.position.z = 0.5 #0.4

    move_group.set_pose_target(pose_goal)

    # 자세(pose) 목표로 로봇 관절 그룹을 움직이도록 go 명령을 호출한다.
    plan = move_group.go(wait=True)
    # 남은 로봇 움직임이 없다는 것을 확인하기 위해 ``stop()``을 호출한다
    move_group.stop()
    # 자세로 플래닝한 후에 목표 지점들을 지운다
    # 주의: 관절 목표에 대한 clear_joint_value_targets() 함수는 없다
    move_group.clear_pose_targets()

    # 최종 로봇 위치 검사
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # 클래스 변수를 지역 변수로 복사
    move_group = self.move_group

    ## 카르테시안(Cartesian) 경로
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 새로운 자세 목표를 지정하면 이전 목표와의 위치 차이값(x, y, z)을 바탕으로
    ## 말단장치가 지나갈 경유지점(waypoint)들을 지정하여
    ## 카르테시안 경로를 계획할 수 있다.
    ## scale = 1.0 으로 설정한다.
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # 첫째 위로 (z)
    wpose.position.y += scale * 0.2  # 그리고 옆으로 움직인다 (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # 둘째 앞/뒤로 움직인다 (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # 셋째 옆으로 움직인다 (y)
    waypoints.append(copy.deepcopy(wpose))

    # 카르테시안 경로를 계산한다
    # eef_step 값을 0.01로 지정하여 1cm 해상도로 보간(interpolate)되도록 한다
    # jump_threshold를 0.0으로 설정하여 관절공간에서 불가능한 점프 체크를 무시
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # 주의: 그저 플래닝만 수행하고 실제로 로봇을 움직이지는 않는다
    return plan, fraction

  def display_trajectory(self, plan):
    # 클래스 변수를 지역 변수로 복사
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## 궤적(Trajectory) 표시
    ## ^^^^^^^^^^^^^^^^^^^^^
    ## RViz에서 플랜(또는 궤적)을 시각화하도록 한다.
    ## group.plan() 메소드는 자동적으로 수행하므로 이 메소드를 사용할 필요없다
    ##
    ## `DisplayTrajectory` 메시지는 trajectory_start와 trajectory의 2개의 항목을 가진다._
    ## 따라서 충돌 객체들이 포함된 현재 로봇 상태를 trajectory_start로 지정하고
    ## 인자로 받은 새로운 플랜을 trajectory에 추가한다.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    # 클래스 변수를 지역 변수로 복사
    move_group = self.move_group

    ## 계획(Plan) 실행
    ## ^^^^^^^^^^^^^^^
    ## 로봇이 이미 계산된 플랜을 따라가도록 실행한다
    move_group.execute(plan, wait=True)

    ## 주의: 로봇의 현재 관절 상태는 RobotTrajectory에서 첫째 경유지점의
    ## 공차 범위 내에 있지 않으면 ``execute()``는 실패할 것이다

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # 클래스 변수를 지역 변수로 복사
    box_name = self.box_name
    scene = self.scene

    ## 충돌 갱신이 되었는지 확인
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 파이썬 노드가 충돌 객체 갱신 메시지를 출판하기 전에 죽으면
    ## 메시지는 유실되고 충돌 박스들은 나타나지 않을 것이다.
    ## 갱신이 되었는지 확인하려면 get_attached_objects()와
    ## get_known_object_names() 함수를 사용하여
    ## 각각 부착 객체 및 인식(known) 객체 목록에 반영되었는지 확인할 때까지
    ## 또는 timeout 초가 지날 때까지 대기한다
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # 박스가 부착된(attached) 객체인지 검사
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

  def add_box(self, timeout=4):
    # 클래스 변수를 지역 변수로 복사
    box_name = self.box_name
    scene = self.scene

    ## 플래닝 씬(Planning Scene)에 객체 추가
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 먼저, 툴 위치에 플래닝 씬에 추가할 박스 생성
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    # 지역 변수를 클래스 변수로 역으로 복사
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # 클래스 변수를 지역 변수로 복사
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## 로봇에 객체 부착
    ## ^^^^^^^^^^^^^^^^^^
    ## 박스를 로봇 몸체에 부착하여 객체에 접촉할 수 있도록 한다
    ## 링크 이름을 ``touch_links`` 배열로 추가하면,
    ## 플래닝 씬은 이 링크와 박스 사이의 충돌을 무시한다
    attached_group = 'manipulator'
    touch_links = robot.get_link_names(group=attached_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    # 플래닝 씬이 갱신되기를 기다린다
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # 클래스 변수를 지역 변수로 복사
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## 로봇으로부터 객체를 탈착
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## 플래닝 씬으로부터 객체를 탈착하고 제거할 수 있다
    scene.remove_attached_object(eef_link, name=box_name)

    # 플래닝 씬이 갱신되기를 기다린다
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # 클래스 변수를 지역 변수로 복사
    box_name = self.box_name
    scene = self.scene

    ## 플래닝 씬으로부터 객체 제거
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 박스를 플래닝 씬으로부터 제거한다
    scene.remove_world_object(box_name)

    ## 주의: 객체는 제거되기 전에 부착되었으면 탈착(detached) 한다
    # 플래닝 씬이 갱신되기를 기다린다
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    raw_input()
    tutorial = MoveArm()

    print("============ Press `Enter` to execute a movement using a joint state goal ...")
    raw_input()
    tutorial.go_to_joint_state()

    print("============ Press `Enter` to execute a movement using a pose goal ...")
    raw_input()
    tutorial.go_to_pose_goal()

    print("============ Press `Enter` to plan and display a Cartesian path ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print("============ Press `Enter` to execute a saved path ...")
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print("============ Press `Enter` to add a box to the planning scene ...")
    raw_input()
    tutorial.add_box()

    print("============ Press `Enter` to attach a Box to the ar1440 robot ...")
    raw_input()
    tutorial.attach_box()

    print("============ Press `Enter` to plan and execute a path with an attached collision object ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    print("============ Press `Enter` to detach the box from the ar1440 robot ...")
    raw_input()
    tutorial.detach_box()

    print("============ Press `Enter` to remove the box from the planning scene ...")
    raw_input()
    tutorial.remove_box()

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
