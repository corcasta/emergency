#!/usr/bin/env python

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

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from math import pi
from std_srvs.srv import Empty
import tf2_ros
import copy

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('hkust_example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander(rospy.get_namespace()+"robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_pose(self, group, pose):
    """
    group: type string, represening a move group commander (arm, gripper)
    pose:  type geometry_msgs.msg.Pose(), representing target pose
    """
    if group == "arm":
      move_group = self.arm_group
    elif group == "gripper":
      move_group = self.gripper_group
    else:
      print("Error: Invalid group name")
      return False

    move_group.set_pose_target(pose)
    trajectory = move_group.plan()
    
    #if success_flag:
    return move_group.execute(trajectory, wait=True)
    #else:
    #  print("Can NOT find a plan for that pose")
    #  return False

  def reach_joint_angles(self, group, joint_positions, tolerance):
      """
      group: type string, represening a move group commander (arm, gripper)
      joint_positions: type list of floats, size equal to the number of joints in the group
      tolerance: type float 
      """
      if group == "arm":
        move_group = self.arm_group
      elif group == "gripper":
        move_group = self.gripper_group
      else:
        print("Error: Invalid group name")
        return False
      
      success = True

      # Get the current joint positions
      current_joint_positions = move_group.get_current_joint_values()
      #rospy.loginfo("Printing current joint positions BEFORE movement :")
      #or p in current_joint_positions: rospy.loginfo(p)

      # Set the goal joint tolerance
      move_group.set_goal_joint_tolerance(tolerance)
      move_group.set_joint_value_target(joint_positions)

      # Plan and execute in one command
      success &= move_group.go(wait=True)

      # Show joint positions after movement
      new_joint_positions = move_group.get_current_joint_values()
      #rospy.loginfo("Printing current joint positions AFTER movement :")
      #for p in new_joint_positions: rospy.loginfo(p)
      return success

  def reach_named_position(self, target):
    """
    target:   type string, referring to a particular save position in configuration
    """
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    success = True
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute in one command
    success &= arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    #rospy.loginfo("Printing current joint positions AFTER movement :")
    #for p in new_joint_positions: rospy.loginfo(p)
    return success

  def cartesian_path(self, target_pose):
    waypoints = []
    wpose = self.arm_group.get_current_pose().pose
    wpose.position.x -= target_pose.position.x
    wpose.position.y -= target_pose.position.y
    wpose.position.z -= target_pose.position.z
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.arm_group.compute_cartesian_path(
                                                              waypoints,   # waypoints to follow
                                                              0.01,        # eef_step
                                                              0.0) 
    self.arm_group.execute(plan, wait=True)

  def reach_gripper_position(self, relative_position):
    """
    relative_position: type float, value between 0-1, where 0 represent completely opened and 1 completely closed
    """
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

  def target_joint_pose(self, group, pose, tolerance):
      if group == "arm":
        move_group = self.arm_group
      elif group == "gripper":
        move_group = self.gripper_group
      else:
        print("Error: Invalid group name")
        return False
      
      success = True

      # Get the current joint positions
      current_joint_positions = move_group.get_current_joint_values()
      rospy.loginfo("Printing current joint positions BEFORE movement :")
      for p in current_joint_positions: rospy.loginfo(p)

      # Set the goal joint tolerance
      move_group.set_goal_joint_tolerance(tolerance)
      move_group.set_joint_value_target(pose, True)

      # Plan and execute in one command
      success &= move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      # Show joint positions after movement
      new_joint_positions = move_group.get_current_joint_values()
      rospy.loginfo("Printing current joint positions AFTER movement :")
      for p in new_joint_positions: rospy.loginfo(p)
      return success

  def set_planner(self, planner_id):
      if planner_id:
          self.arm_group.set_planner_id(planner_id)
          self.gripper_group.set_planner_id(planner_id)

def grasp_decorator(tf_buffer, pose, target_frame):
  """
  target_frame: type string, representing the frame pose we want to move on (e.g. pre_grasp_position)
  """
  #tf_buffer = buffer
  def pre_grasp_callback(pose_stamped):
    """
    pose_stamped: type PoseStamped msg
    """
    #global target_pose
    #tf_buffer.can_transform('base_link', 'pre_grasp_position', rospy.Time(0), rospy.Duration(4.0))
    transform = tf_buffer.lookup_transform('base_link', target_frame, rospy.Time())
    #print(transform)
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z

    transform_data = tf_buffer.lookup_transform('base_link', 'marker_prime', rospy.Time())
    pose.orientation.x = transform_data.transform.rotation.x
    pose.orientation.y = transform_data.transform.rotation.y
    pose.orientation.z = transform_data.transform.rotation.z
    pose.orientation.w = transform_data.transform.rotation.w
  return pre_grasp_callback


def stop_decorator(group, emergency_stop):
  def stop_callback(bool_msg):
    if bool_msg.data == True:
      print("Stopping Arm Movement")
      group.stop()
      print("Arm should be completely stop")
      emergency_stop.data = True
      
  return stop_callback


def main():
  """
    - SBL
    - EST
    - LBKPIECE +
    - BKPIECE
    - KPIECE
    - RRT
    - RRTConnect
    - RRTstar
    - TRRT
    - PRM
    - PRMstar
    - FMT
    - BFMT -
    - PDST
    - STRIDE
    - BiTRRT
    - LBTRRT
    - BiEST
    - ProjEST
    - LazyPRM
    - LazyPRMstar
    - SPARS
    - SPARStwo
  """
  planner = "RRTConnect"
  example = ExampleMoveItTrajectories()
  example.set_planner(planner)
  #***********************************************************
  #***********************************************************
  waiting_time = 2
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  
  pre_grasp_pose = Pose()
  pre_grasp_callback = grasp_decorator(tfBuffer, pre_grasp_pose, "pre_grasp_position")
  rospy.Subscriber("/aruco_single/pose", PoseStamped, pre_grasp_callback)

  grasp_pose = Pose()
  grasp_callback = grasp_decorator(tfBuffer, grasp_pose, "max_grasp_position")
  rospy.Subscriber("/aruco_single/pose", PoseStamped, grasp_callback)

  sudden_stop = std_msgs.msg.Bool()
  stop_callback = stop_decorator(example.arm_group, sudden_stop)
  
  #***********************************************************
  #***********************************************************

  activate_subscription = True

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  
  if success:
    print("INSIDE SUCCESS")

    pose_vector = 0
    state = "HOME_POSITION"

    while not rospy.is_shutdown():
      #*************************************************************************************************************************
      if state == "HOME_POSITION":
        home_pose =  [0,
                      -1.710422667,
                      -1.3613568166,
                      -3.1066860685,
                      1.8849555922,
                      -1.5707963268]
        process = example.reach_joint_angles("arm", home_pose, tolerance=0.001)
        if process:
          state = "PRE_GRASP_POSITION"
        else:
          state = "HOME_POSITION"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "PRE_GRASP_POSITION":
        # This state as the name implies will make 
        # the arm move to the pre-grasp position
        for _ in range(2):
          print("Loop")
          new_pose_vect = math.sqrt(pre_grasp_pose.position.x**2 + pre_grasp_pose.position.y**2 + pre_grasp_pose.position.z**2)
          if abs(pose_vector-new_pose_vect) > 0.001:
            print("Moving to PRE_GRASP_POSITION")  
            #example.reach_pose("arm", pre_grasp_pose)
            process = example.reach_cartesian_pose(pre_grasp_pose, tolerance=0.01, constraints=None)
            print("It is moving")
            #process = example.target_joint_pose("arm", pre_grasp_pose, tolerance=0.01)
            print("Success? ", process)
            rospy.sleep(waiting_time)

            tcp_pose = example.arm_group.get_current_pose() # Tool frame pose wrt base_link
            pose_vector = math.sqrt(tcp_pose.pose.position.x**2 + tcp_pose.pose.position.y**2 + tcp_pose.pose.position.z**2)
            # If the robot arm reached target position we can  
            # move to the next state in the sequence else we 
            # stay in the same state until we are completely done

            if process:
              state = "GRASP_POSITION"
              pre_grasp_pose_saved = pre_grasp_pose
            else:
              state = "PRE_GRASP_POSITION"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "GRASP_POSITION":
        # This state as the name implies will make 
        # the arm move to the grasp position
        if activate_subscription:
          disk_detector = rospy.Subscriber("/detector1", std_msgs.msg.Bool, stop_callback)
          activate_subscription = False

        new_pose_vect = math.sqrt(grasp_pose.position.x**2 + grasp_pose.position.y**2 + grasp_pose.position.z**2)
        if abs(pose_vector-new_pose_vect) > 0.01:
          print("Moving to GRASP_POSITION")  
          #example.reach_pose("arm", grasp_pose)
          print(grasp_pose)
          process = example.reach_cartesian_pose(grasp_pose, tolerance=0.001, constraints=None)
          #process = example.target_joint_pose("arm", grasp_pose, tolerance=0.01)
          print("Success? ", process)
          rospy.sleep(waiting_time)

          tcp_pose = example.arm_group.get_current_pose() # Tool frame pose wrt base_link
          pose_vector = math.sqrt(tcp_pose.pose.position.x**2 + tcp_pose.pose.position.y**2 + tcp_pose.pose.position.z**2)
          # If the robot arm reached target position we can  
          # move to the next state in the sequence else we 
          # stay in the same state until we are completely done
          
          #print("SUDDEN STOP: ", sudden_stop.data)
          if sudden_stop.data:
            # We are expecting the gripper to detect the red line
            # somewhere in between the trayectory if that doesnt happened
            # go back to pre-grasp and repeat the motion to the grasp position
            # until the red line is detected. 
            disk_detector.unregister()
            #example.arm_group.forget_joint_values()
            #example.arm_group.set_start_state(example.robot.get_current_state())
            #example.arm_group.clear_pose_target("end_effector_link")
            #example.gripper_group.clear_pose_target("end_effector_link")
            #example.arm_group.set_start_state_to_current_state()
            state = "CLOSE_GRIPPER"
          else:
            state = "PRE_GRASP_POSITION"
       #*************************************************************************************************************************   
      
      #*************************************************************************************************************************
      elif state == "CLOSE_GRIPPER":
        # This state as the name implies will make 
        # the arm move to the CLOSE_GRIPPER position
        print("CLOSE GRIPPER")
        # preset width=0.85, if the tactile feedback is not True, width+0.05,, until the feedback is True
        """
          width = 0.85
          process = example.reach_gripper_position(width)

          def gripper_stop_callback(tactile_msg):
            global width,process

            if tactile_msg.data is not True:
              width += 0.005
              process = example.reach_gripper_position(width)
          rospy.Subscriber("/tactile_msg1",std_msgs.msg.Bool,gripper_stop_callback)
          """
        ###
        process = example.reach_gripper_position(0.85)
        print("Process: ", process)
        if process:
          state = "POST_GRASP_POSITION"
        else:
          state = "CLOSE_GRIPPER"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "POST_GRASP_POSITION":
        # This state as the name implies will make 
        # the arm move to the POST_GRASP_POSITION
        new_pose_vect = math.sqrt(pre_grasp_pose_saved.position.x**2 + pre_grasp_pose_saved.position.y**2 + pre_grasp_pose_saved.position.z**2)
        if abs(pose_vector-new_pose_vect) > 0.01:
          print("Moving to POST_GRASP_POSITION")  
          #example.reach_pose("arm", pre_grasp_pose)
          process = example.reach_cartesian_pose(pre_grasp_pose_saved, tolerance=0.001, constraints=None)
          #process = example.target_joint_pose("arm", pre_grasp_pose_saved, tolerance=0.01)
          print("Success? ", process)
          rospy.sleep(waiting_time)

          tcp_pose = example.arm_group.get_current_pose() # Tool frame pose wrt base_link
          pose_vector = math.sqrt(tcp_pose.pose.position.x**2 + tcp_pose.pose.position.y**2 + tcp_pose.pose.position.z**2)
          # If the robot arm reached target position we can  
          # move to the next state in the sequence else we 
          # stay in the same state until we are completely done
          if process:
            state = "PRE_BASKET_POSITION"
          else:
            state = "POST_GRASP_POSITION"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "PRE_BASKET_POSITION":
        print("Moving to PRE_BASKET_POSITION")
        post_grasp_pose =  [-0.0872664626,
                            -0.872664626,
                            0.2617993878,
                            -3.054326191,
                            -1.745329252,
                            3.1066860685]
        process = example.reach_joint_angles("arm", post_grasp_pose, tolerance=0.001)
        print("Success? ", process)
        rospy.sleep(waiting_time)
        
        if process:
          state = "BASKET_POSITION"
        else:
          state = "PRE_BASKET_POSITION"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "BASKET_POSITION":
        print("Moving to BASKET_POSITION")
        post_grasp_pose =  [-0.0698131701,
                            -0.9599310886,
                            0.4886921906,
                            -3.0717794835,
                            -1.5358897418,
                            3.1066860685]
        process = example.reach_joint_angles("arm", post_grasp_pose, tolerance=0.001)
        print("Success? ", process)
        rospy.sleep(waiting_time)

        if process:
          state = "OPEN_GRIPPER"
        else:
          state = "BASKET_POSITION"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "OPEN_GRIPPER":
        # This state as the name implies will make 
        # the arm move to the CLOSE_GRIPPER position
        print("OPEN GRIPPER")
        # preset width=0.85, if the tactile feedback is not True, width+0.05,, until the feedback is True

        ###
        process = example.reach_gripper_position(0.2)
        print("Process: ", process)
        
        if process:
          state = "POST_BASKET_POSITION"
        else:
          state = "OPEN_GRIPPER"
      #*************************************************************************************************************************
      
      #*************************************************************************************************************************
      elif state == "POST_BASKET_POSITION":
        print("Moving to POST_BASKET_POSITION")
        post_grasp_pose =  [-0.0872664626,
                            -0.872664626,
                            0.2617993878,
                            -3.054326191,
                            -1.745329252,
                            3.1066860685]
                            
        process = example.reach_joint_angles("arm", post_grasp_pose, tolerance=0.001)
        print("Success? ", process)
        rospy.sleep(waiting_time)

        if process:
          state = "END_POSITION"
        else:
          state = "POST_BASKET_POSITION"
      #*************************************************************************************************************************

      #*************************************************************************************************************************
      elif state == "END_POSITION":
        print("Moving to END_POSITION")
        end_pose =  [0,
                      -1.710422667,
                      -1.3613568166,
                      -3.1066860685,
                      1.8849555922,
                      -1.5707963268]
        process = example.reach_joint_angles("arm", end_pose, tolerance=0.001)
        print("Success? ", process)
        rospy.sleep(waiting_time)

        if process:
          state = "PRE_GRASP_POSITION"
          print("Done :)")
          break
        else:
          state = "END_POSITION"
      #*************************************************************************************************************************
  if not success:
      rospy.logerr("The example encountered an error.")


if __name__ == '__main__':
  main()
