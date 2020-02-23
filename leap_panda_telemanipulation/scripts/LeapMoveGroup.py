#!/usr/bin/env python
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from leap_panda_telemanipulation.msg import Modified_leap
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import Tkinter
from Tkinter import *
import math

def all_close(goal, actual, tolerance):
  all_equal = True

  if type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  
  return True

#code to convert roll pitch yaw into quaternion
def Quaternion(pose, roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    
    pose.orientation.w = cy * cp * cr + sy * sp * sr
    #pose.orientation.x = cy * cp * sr - sy * sp * cr
    #pose.orientation.y = sy * cp * sr + cy * sp * cr
    #pose.orientation.z = sy * cp * cr - cy * sp * sr

    return pose

class LeapMoveGroup(object):

  def __init__(self):
    super(LeapMoveGroup, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('LeapMoveGroup', anonymous=True)
    arm = moveit_commander.MoveGroupCommander("panda_arm")

    # Misc variables
    self.subscriber = None
    self.arm = arm
    self.space_modifier = 1
    self.offset_x = 0
    self.offset_y = 0
    self.offset_z = 0
    self.waiting = False
    
    #take in Modified_leap.msg
    self.leap = Modified_leap()

    self.setupGui()


    print("Staring subscriber")
    subscriber = rospy.Subscriber("/leap_to_panda", Modified_leap, self.callback, queue_size=1)
    
    self.waiting = TRUE
    print("Staring gui")
    self.leap_window.mainloop()
  
  def setupGui(self):
    #diagnostic gui
    self.leap_window = Tkinter.Tk()
    self.leap_window.title('Tracking Info')

    #left hand
    self.leap_lefthand_label_hand = Label(self.leap_window, text='Left Hand').grid(row=0, column=1)
    self.gui_left_x = StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_x = Label(self.leap_window, text='X:').grid(row=1, column=0) 
    self.leap_lefthand_label_xv = Label(self.leap_window, textvariable=self.gui_left_x).grid(row=1, column=1)
    self.gui_left_y = StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_y = Label(self.leap_window, text='Y:').grid(row=2, column=0) 
    self.leap_lefthand_label_yv = Label(self.leap_window, textvariable=self.gui_left_y).grid(row=2, column=1)
    self.gui_left_z = StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_z = Label(self.leap_window, text='Z:').grid(row=3, column=0) 
    self.leap_lefthand_label_zv = Label(self.leap_window, textvariable=self.gui_left_z).grid(row=3, column=1) 
    self.gui_left_roll = StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_roll = Label(self.leap_window, text='Roll:').grid(row=4, column=0) 
    self.leap_lefthand_label_rollv = Label(self.leap_window, textvariable=self.gui_left_roll).grid(row=4, column=1)
    self.gui_left_pitch = StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_pitch = Label(self.leap_window, text='Pitch:').grid(row=5, column=0) 
    self.leap_lefthand_label_pitchv = Label(self.leap_window, textvariable=self.gui_left_pitch).grid(row=5, column=1)
    self.gui_left_yaw = StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_yaw = Label(self.leap_window, text='Yaw:').grid(row=6, column=0) 
    self.leap_lefthand_label_yawv = Label(self.leap_window, textvariable=self.gui_left_yaw).grid(row=6, column=1) 
    #self.leap_lefthand_label_ges(self.leap_window, text='Gesture').grid(row=0, column=7) 
    #self.leap_lefthand_label_cert(self.leap_window, text='Certainty').grid(row=0, column=8)

    #right hand gui
    self.leap_righthand_label_hand = Label(self.leap_window, text='Right Hand').grid(row=0, column=2) 
    self.gui_right_x = StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_xv = Label(self.leap_window, textvariable=self.gui_right_x).grid(row=1, column=2)
    self.gui_right_y  = StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_yv = Label(self.leap_window, textvariable=self.gui_right_y).grid(row=2, column=2)
    self.gui_right_z = StringVar(self.leap_window, "0.00 ") 
    self.leap_righthand_label_zv = Label(self.leap_window, textvariable=self.gui_right_z).grid(row=3, column=2) 
    self.gui_right_roll = StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_rollv = Label(self.leap_window, textvariable=self.gui_right_roll).grid(row=4, column=2) 
    self.gui_right_pitch = StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_pitchv = Label(self.leap_window, textvariable=self.gui_right_pitch).grid(row=5, column=2) 
    self.gui_right_yaw = StringVar(self.leap_window, "0.00 ")
    self.leap_righthand_label_yawv = Label(self.leap_window, textvariable=self.gui_right_yaw).grid(row=6, column=2) 
    #self.leap_righthand_label_ges(self.leap_window, text='Gesture').grid(row=1, column=7) 
    #self.leap_righthand_label_cert(self.leap_window, text='Certainty').grid(row=1, column=8)

    #panda gui
    #self.panda_label_panda(self.leap_window, text='Panda').grid(row=2, column=0) 
    #self.panda_label_other(self.leap_window, text='tba').grid(row=2, column=1) 

    #offset
    self.leap_offset_label = Label(self.leap_window, text='Offset').grid(row=7, column=0) 
    self.leap_offset_x = StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_x = Label(self.leap_window, text='X:').grid(row=8, column=0) 
    self.leap_offset_label_xv = Label(self.leap_window, textvariable=self.leap_offset_x).grid(row=8, column=1) 
    self.leap_offset_y = StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_y = Label(self.leap_window, text='Y:').grid(row=9, column=0) 
    self.leap_offset_label_yv = Label(self.leap_window, textvariable=self.leap_offset_y).grid(row=9, column=1) 
    self.leap_offset_z = StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_z = Label(self.leap_window, text='Z:').grid(row=10, column=0) 
    self.leap_offset_label_zv = Label(self.leap_window, textvariable=self.leap_offset_z).grid(row=10, column=1) 
    
    #input scale
    self.gui_scale = StringVar(self.leap_window, "0.00 ")
    self.leap_label_scale = Label(self.leap_window, text='Scale:').grid(row=11, column=0) 
    self.leap_label_scalev = Label(self.leap_window, textvariable=self.gui_scale).grid(row=11, column=1) 

    self.reset_button = Button(self.leap_window, text = "Reset", command = self.reset_pos).grid(row=12,column=1)

    self.gui_scale.set(self.space_modifier)

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

    self.leap_offset_x .set(self.offset_x)
    self.leap_offset_y.set(self.offset_y) 
    self.leap_offset_z.set(self.offset_z)

  def doubleToString(self, input):
    return round(input, 2)

  def callback(self, data):
    self.leap = data
    self.updateGui()

    if self.waiting:
      self.waiting = False
      
      self.go_to_pose_goal()   
      
      self.waiting = True

  def reset_pos(self):
    print("Resetting")
    current_joints = self.arm.get_current_joint_values()
    joint_goal = self.arm.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -math.pi/4
    joint_goal[2] = 0
    joint_goal[3] = -math.pi/2
    joint_goal[4] = 0
    joint_goal[5] = math.pi/3
    joint_goal[6] = 0

    arm_plan = self.arm.go(wait=True)
    self.arm.stop() 
    return all_close(joint_goal, current_joints, 0.001)

  def go_to_pose_goal(self):    
    ## Planning to a Pose Goal
    ## We can plan a motion for this arm to a desired pose for the    
    arm_pose_goal = geometry_msgs.msg.Pose()
    #arm_pose_goal = Quaternion(arm_pose_goal,self.leap.right_orientation[0],self.leap.right_orientation[1],self.leap.right_orientation[2])
    arm_pose_goal.position.x = ((self.leap.right_location[0])+self.offset_x)*self.space_modifier
    arm_pose_goal.position.z = ((self.leap.right_location[1])+self.offset_y)*self.space_modifier
    arm_pose_goal.position.y = ((-1*self.leap.right_location[2])+self.offset_z)*self.space_modifier

    print ("")
    print ("Moving to:")
    print (arm_pose_goal.position.x + arm_pose_goal.position.y + arm_pose_goal.position.z)
    print (arm_pose_goal.orientation.w + arm_pose_goal.orientation.x + arm_pose_goal.orientation.y + arm_pose_goal.orientation.z)
    print ("")

    self.arm.set_pose_target(arm_pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    arm_plan = self.arm.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.arm.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm.clear_pose_targets()

    current_arm_pose = self.arm.get_current_pose().pose

    return all_close(arm_pose_goal, current_arm_pose, 0.001)

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
