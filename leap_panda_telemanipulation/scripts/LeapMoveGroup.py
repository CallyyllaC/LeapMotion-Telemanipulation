#!/usr/bin/env python
import math
import sys
import Tkinter

import geometry_msgs.msg
import moveit_commander
import numpy as np
import rospy
import tf.transformations
import transforms3d as tr
from leap_panda_telemanipulation.msg import Modified_leap
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String

#from scipy.spatial.transform import Rotation as R

def all_close(goal, actual, tolerance):
  all_equal = True

  if type(goal) is list:
    for i in range(len(goal)):
      if abs(actual[i] - goal[i])>tolerance:
        return False

  if type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  
  return True


class LeapMoveGroup(object):

  def __init__(self):
    super(LeapMoveGroup, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('LeapMoveGroup', anonymous=True)
    robot = moveit_commander.RobotCommander()
    print ""
    print "panda arm hand group joints"
    print robot.get_joint_names("panda_arm_hand")
    arm = moveit_commander.MoveGroupCommander("panda_arm_hand")

    # Misc variables
    self.subscriber = None
    self.arm = arm
    self.space_modifier = 2
    self.tolerance = 0.01
    self.previous_offset = [0,0,0]
    self.offset_x = 0
    self.offset_y = 0
    self.offset_z = 0
    self.waiting = False
    self.default_pose = None
    
    #take in Modified_leap.msg
    self.leap = Modified_leap()

    self.setupGui()


    print("Staring subscriber")
    subscriber = rospy.Subscriber("/leap_to_panda", Modified_leap, self.callback, queue_size=1)

    self.default_pose = None
    
    self.waiting = True
    print("Staring gui")
    self.leap_window.mainloop()
  
  def setupGui(self):
    #diagnostic gui
    self.leap_window = Tkinter.Tk()
    self.leap_window.title('Tracking Info')

    #left hand
    self.leap_lefthand_label_hand = Tkinter.Label(self.leap_window, text='Left Hand').grid(row=0, column=1)
    self.gui_left_x = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_x = Tkinter.Label(self.leap_window, text='X:').grid(row=1, column=0) 
    self.leap_lefthand_label_xv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_x).grid(row=1, column=1)
    self.gui_left_y = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_y = Tkinter.Label(self.leap_window, text='Y:').grid(row=2, column=0) 
    self.leap_lefthand_label_yv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_y).grid(row=2, column=1)
    self.gui_left_z = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_z = Tkinter.Label(self.leap_window, text='Z:').grid(row=3, column=0) 
    self.leap_lefthand_label_zv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_z).grid(row=3, column=1) 
    self.gui_left_roll = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_roll = Tkinter.Label(self.leap_window, text='Roll:').grid(row=4, column=0) 
    self.leap_lefthand_label_rollv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_roll).grid(row=4, column=1)
    self.gui_left_pitch = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_pitch = Tkinter.Label(self.leap_window, text='Pitch:').grid(row=5, column=0) 
    self.leap_lefthand_label_pitchv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_pitch).grid(row=5, column=1)
    self.gui_left_yaw = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_yaw = Tkinter.Label(self.leap_window, text='Yaw:').grid(row=6, column=0) 
    self.leap_lefthand_label_yawv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_yaw).grid(row=6, column=1) 
    self.gui_left_grab = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_grab = Tkinter.Label(self.leap_window, text='Gesture:').grid(row=7, column=0) 
    self.leap_lefthand_label_grabv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_grab).grid(row=7, column=1) 

    #right hand gui
    self.leap_righthand_label_hand = Tkinter.Label(self.leap_window, text='Right Hand').grid(row=0, column=2) 
    self.gui_right_x = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_xv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_x).grid(row=1, column=2)
    self.gui_right_y  = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_yv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_y).grid(row=2, column=2)
    self.gui_right_z = Tkinter.StringVar(self.leap_window, "0.00 ") 
    self.leap_righthand_label_zv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_z).grid(row=3, column=2) 
    self.gui_right_roll = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_rollv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_roll).grid(row=4, column=2) 
    self.gui_right_pitch = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_pitchv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_pitch).grid(row=5, column=2) 
    self.gui_right_yaw = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_yawv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_yaw).grid(row=6, column=2) 
    self.gui_right_pinch = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_pinchv = Tkinter.Label(self.leap_window, textvariable=self.gui_right_pinch).grid(row=7, column=2) 

    #offset
    self.leap_offset_label = Tkinter.Label(self.leap_window, text='Offset').grid(row=8, column=0) 
    self.leap_offset_x = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_x = Tkinter.Label(self.leap_window, text='X:').grid(row=9, column=0) 
    self.leap_offset_label_xv = Tkinter.Label(self.leap_window, textvariable=self.leap_offset_x).grid(row=9, column=1) 
    self.leap_offset_y = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_y = Tkinter.Label(self.leap_window, text='Y:').grid(row=10, column=0) 
    self.leap_offset_label_yv = Tkinter.Label(self.leap_window, textvariable=self.leap_offset_y).grid(row=10, column=1) 
    self.leap_offset_z = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_z = Tkinter.Label(self.leap_window, text='Z:').grid(row=11, column=0) 
    self.leap_offset_label_zv = Tkinter.Label(self.leap_window, textvariable=self.leap_offset_z).grid(row=11, column=1) 
    
    self.reset_button = Tkinter.Button(self.leap_window, text = "Reset Offset", command = self.reset_off).grid(row=12,column=1)

    #input scale
    self.gui_scale = Tkinter.StringVar(self.leap_window, self.space_modifier)
    self.leap_label_scale = Tkinter.Label(self.leap_window, text='Scale:').grid(row=13, column=0) 
    self.leap_label_scalev = Tkinter.Entry(self.leap_window, textvariable=self.gui_scale).grid(row=13, column=1)

    #movement tolerance
    self.gui_tolerance = Tkinter.StringVar(self.leap_window, self.space_modifier)
    self.leap_label_tolerance = Tkinter.Label(self.leap_window, text='Tolerance:').grid(row=14, column=0) 
    self.leap_label_tolerancev = Tkinter.Entry(self.leap_window, textvariable=self.gui_tolerance).grid(row=14, column=1)

    self.reset_button = Tkinter.Button(self.leap_window, text = "Reset Robot", command = self.reset_pos).grid(row=15,column=1)

    self.gui_scale.set(self.space_modifier)
    self.gui_tolerance.set(self.tolerance)

  def updateGui(self):
    self.gui_left_x.set(self.doubleToString(self.leap.left_location[0]))
    self.gui_left_y.set(self.doubleToString(self.leap.left_location[1]))
    self.gui_left_z.set(self.doubleToString(self.leap.left_location[2]))
    self.gui_left_roll.set(self.doubleToString(self.leap.left_orientation[0]))
    self.gui_left_pitch.set(self.doubleToString(self.leap.left_orientation[1]))
    self.gui_left_yaw.set(self.doubleToString(self.leap.left_orientation[2]))
    self.gui_right_x.set(self.doubleToString(self.leap.right_location[0]))
    self.gui_right_y.set(self.doubleToString(self.leap.right_location[1]))
    self.gui_right_z.set(self.doubleToString(self.leap.right_location[2]))
    self.gui_right_roll.set(self.doubleToString(self.leap.right_orientation[0]))
    self.gui_right_pitch.set(self.doubleToString(self.leap.right_orientation[1]))
    self.gui_right_yaw.set(self.doubleToString(self.leap.right_orientation[2]))

    self.gui_left_grab.set(self.doubleToString(self.leap.grab))
    self.gui_right_pinch.set(self.doubleToString(self.leap.pinch))

    self.leap_offset_x .set(self.offset_x)
    self.leap_offset_y.set(self.offset_y) 
    self.leap_offset_z.set(self.offset_z)
    self.space_modifier = float(self.gui_scale.get())
    self.tolerance = float(self.gui_tolerance.get())
  
  def doubleToString(self, input):
    return round(input, 2)

  #code to convert roll pitch yaw into quaternion
  def Quaternion(self, pose, roll, pitch, yaw):

    ##tr quarternion accepts pose orientation x,y,z,w format
    #tmp2 = tr.euler.euler2quat(roll, pitch, yaw)
    #pose.orientation = geometry_msgs.msg.Quaternion(*tmp2)
    #flatmat = tr.quaternions.quat2mat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    #print "default pose as quaternion (x,y,z,w): ", pose.orientation.x, " , ", pose.orientation.y, " , ", pose.orientation.z, " , ", pose.orientation.w
    #print "default pose as eular: ", roll,  " , ",pitch,  " , ",yaw
    #print "default pose as rotational matrix: ", flatmat

    #rm = np.array([[1, 0, 0],[0, 0, 1],[0, -1, 0]])
    #tmp = tr.quaternions.mat2quat(rm)
    #print "tmp: ",tmp
    #print "tmp2: ",tmp2
    #out = tr.quaternions.qmult(tmp, tmp2)
    #print "out: ", out
    #pose.orientation = geometry_msgs.msg.Quaternion(*out)
    #print "pose: ", pose.orientation

    
    #make the orientation the same as the default pose
    pose.orientation = self.default_pose.pose.orientation

    return pose
    
  def callback(self, data):
    self.leap = data
    self.updateGui()

    if(self.default_pose == None):
      self.default_pose = self.arm.get_current_pose("panda_link8")

    if self.waiting:
      self.waiting = False
      
      if((self.leap.left_location[0]+self.leap.left_location[1]+self.leap.left_location[2])!=0):
        self.move_workspace()

      elif((self.leap.right_location[0]+self.leap.right_location[1]+self.leap.right_location[2])!=0):
        self.go_to_pinch_goal()
        self.go_to_pose_goal()
      

      self.waiting = True

  def reset_pos(self):
    self.waiting = False
    print("Resetting robot")
    
    current_arm_pose = self.arm.get_current_pose("panda_link8")
    self.arm.set_pose_target(self.default_pose, "panda_link8")

    self.arm.go(wait=True)
    self.arm.stop()
    self.waiting = True
    return all_close(self.default_pose, current_arm_pose, self.tolerance)

  def reset_off(self):
    self.waiting = False
    print("Resetting offset")
    self.offset_x = 0
    self.offset_y = 0
    self.offset_z = 0

  def move_workspace(self):
    if(self.leap.grab > 0.9):
      self.offset_x = self.offset_x + (self.previous_offset[0] - self.leap.left_location[0])
      self.offset_y = self.offset_y + (-1*(self.previous_offset[1] - self.leap.left_location[1]))
      self.offset_z = self.offset_z + (self.previous_offset[2] - self.leap.left_location[2])

      print ("")
      print ("Moving workspace: grip = ", self.leap.grab)
      print ("From:", self.previous_offset[0], self.previous_offset[1], self.previous_offset[2])
      print ("To:", self.leap.left_location[0], self.leap.left_location[1], self.leap.left_location[2])
      print ("")

      self.previous_offset = self.leap.left_location

  def go_to_pose_goal(self):     
    arm_pose_goal = geometry_msgs.msg.Pose()
    arm_pose_goal = self.Quaternion(arm_pose_goal,self.leap.right_orientation[0],self.leap.right_orientation[1],self.leap.right_orientation[2])
    arm_pose_goal.position.x = ((self.leap.right_location[0])+self.offset_x)*self.space_modifier
    arm_pose_goal.position.z = ((self.leap.right_location[1])+self.offset_y)*self.space_modifier
    arm_pose_goal.position.y = -1*((self.leap.right_location[2])+self.offset_z)*self.space_modifier

    print ("")
    print ("Moving to:")
    print (arm_pose_goal.position.x, arm_pose_goal.position.y, arm_pose_goal.position.z)
    print (arm_pose_goal.orientation.w, arm_pose_goal.orientation.x, arm_pose_goal.orientation.y, arm_pose_goal.orientation.z)
    print ("")

    self.arm.set_pose_target(arm_pose_goal, "panda_link8")
    
    self.arm.go(wait=True)

    self.arm.stop()

    self.arm.clear_pose_targets()
    current_arm_pose = self.arm.get_current_pose("panda_link8").pose

    return all_close(arm_pose_goal, current_arm_pose, self.tolerance)

  def go_to_pinch_goal(self):    
    
    print ("")
    print ("Pinching:", self.leap.pinch)


    hand_joint_goal = self.arm.get_current_joint_values()
    
    print hand_joint_goal
    print ("")

    hand_joint_goal[7] = 0.035 - (self.leap.pinch*0.035)
    hand_joint_goal[8] = 0.035 - (self.leap.pinch*0.035)

    self.arm.go(hand_joint_goal, wait=True)

    self.arm.stop()

    current_hand_joints = self.arm.get_current_joint_values()

    return all_close(hand_joint_goal, current_hand_joints, self.tolerance)

def main():
  try:
    test = LeapMoveGroup()
    print ("Script complete, press any key to close")
    raw_input()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()