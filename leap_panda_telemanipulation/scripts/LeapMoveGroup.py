#!/usr/bin/env python
# coding: utf-8

#References
#Docs.ros.org. (2016). Move Group Python Interface — Moveit_Tutorials Kinetic Documentation. [online] Available at: https://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html [Accessed 24 Jan. 2020].
#GitHub. (2013). ros-drivers/leap_motion. [online] Available at: https://github.com/ros-drivers/leap_motion/tree/hydro/ [Accessed 25 Jan. 2020].
#Wiki.python.org. (2019). Tkinter - Python Wiki. [online] Available at: https://wiki.python.org/moin/TkInter [Accessed 4 February 2020].
#GitHub. (2019). Matthew-Brett/Transforms3d. [online] Available at: https://github.com/matthew-brett/transforms3d [Accessed 28 February 2020].
#Ben-Ari, M., 2018. A Tutorial On Euler Angles And Quaternions. 2nd ed. [ebook] Available at: https://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf [Accessed 3 March 2020].

import math
import sys
import Tkinter
import geometry_msgs.msg
import moveit_commander
import rospy
import transforms3d as tr
import numpy as np
from leap_panda_telemanipulation.msg import Modified_leap
from moveit_commander.conversions import pose_to_list
import tf

#All close function for the robot arm
#converts pose into joint values, if the joint values arnt within tolerance return false (Docs.ros.org, 2016)
def all_close(goal, actual, tolerance):
  all_equal = True

  if type(goal) is list:
    for i in range(len(goal)):
      if abs(actual[i] - goal[i])>tolerance:
        return False

  if type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  
  return True


#My move group object
class LeapMoveGroup(object):

  def __init__(self):
    super(LeapMoveGroup, self).__init__()

    #Set up ROS and MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('LeapMoveGroup', anonymous=True)
    robot = moveit_commander.RobotCommander()

    #debug - print out the detected panda joints
    print ""
    print "panda arm hand group joints"
    print robot.get_joint_names("panda_arm_hand")
    #assign the move group commander
    self.arm = moveit_commander.MoveGroupCommander("panda_arm_hand")

    #class variables
    self.subscriber = None            #holds subscriber
    self.space_modifier = 2           #holds the output scale
    self.tolerance = 0.01             #holds the all close tolerance
    self.previous_offset = [0,0,0]    #used to work out the current offset
    self.offset_x = 0                 #holds the x value offset
    self.offset_y = 0                 #holds the y value offset
    self.offset_z = 0                 #holds the z value offset
    self.waiting = False              #tells the callback if we are waiting to send a new command to the robot
    self.default_pose = None          #holds the default pose of the panda arm
    self.orientation_mode = False     #Experimental --- tells us if we are using the hand messages orientation for the panda arm
    self.calibration = 0              #holds the current calibration distance when using multiple leap devices, if only using a single device this should be 0
    self.leap = Modified_leap()       #holds the message from leap to panda subscriber

    #set up the GUI
    self.setupGui()

    #Subscribe to LeaptoPanda.cpp output
    print("Staring subscriber")
    self.subscriber = rospy.Subscriber("/leap_to_panda", Modified_leap, self.callback, queue_size=1)
    
    #init complete, ready for some data
    self.waiting = True

    #start the gui and use this to keep the program running so there is no need for rospy.spin()
    print("Staring gui")
    self.leap_window.mainloop()
  
  #Set up the GUI
  def setupGui(self):
    #this is a diagnostic gui used for more detailed information about the input and what is happening to the output
    self.leap_window = Tkinter.Tk()
    self.leap_window.title('Tracking Info')

    #set up the labels for the gui to give context to the data
    self.leap_lefthand_label_hand = Tkinter.Label(self.leap_window, text='Left Hand').grid(row=0, column=1)
    self.leap_righthand_label_hand = Tkinter.Label(self.leap_window, text='Right Hand').grid(row=0, column=2) 
    self.leap_lefthand_label_x = Tkinter.Label(self.leap_window, text='X:').grid(row=1, column=0)
    self.leap_lefthand_label_y = Tkinter.Label(self.leap_window, text='Y:').grid(row=2, column=0) 
    self.leap_lefthand_label_z = Tkinter.Label(self.leap_window, text='Z:').grid(row=3, column=0) 
    self.leap_lefthand_label_roll = Tkinter.Label(self.leap_window, text='Roll:').grid(row=4, column=0)
    self.leap_lefthand_label_pitch = Tkinter.Label(self.leap_window, text='Pitch:').grid(row=5, column=0) 
    self.leap_lefthand_label_yaw = Tkinter.Label(self.leap_window, text='Yaw:').grid(row=6, column=0) 
    self.leap_lefthand_label_grab = Tkinter.Label(self.leap_window, text='Gesture:').grid(row=7, column=0)
    self.leap_offset_label = Tkinter.Label(self.leap_window, text='Offset').grid(row=8, column=0) 
    self.leap_label_scale = Tkinter.Label(self.leap_window, text='Scale:').grid(row=13, column=0) 
    self.leap_label_tolerance = Tkinter.Label(self.leap_window, text='Tolerance:').grid(row=14, column=0) 


    #set up the data for the leap, usually in the format:
    # Tkinter dynamic variable()
    # set variable to a Label()

    #left hand x,y,z pos + r,p,y orientation + grab strength
    self.gui_left_x = Tkinter.StringVar(self.leap_window, "0.00 ") 
    self.leap_lefthand_label_xv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_x).grid(row=1, column=1)
    self.gui_left_y = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_yv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_y).grid(row=2, column=1)
    self.gui_left_z = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_zv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_z).grid(row=3, column=1) 
    self.gui_left_roll = Tkinter.StringVar(self.leap_window, "0.00 ") 
    self.leap_lefthand_label_rollv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_roll).grid(row=4, column=1)
    self.gui_left_pitch = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_pitchv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_pitch).grid(row=5, column=1)
    self.gui_left_yaw = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_yawv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_yaw).grid(row=6, column=1) 
    self.gui_left_grab = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_lefthand_label_grabv = Tkinter.Label(self.leap_window, textvariable=self.gui_left_grab).grid(row=7, column=1) 

    #right hand x,y,z pos + r,p,y orientation + pinch strength
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

    #offset x,y,z
    self.leap_offset_x = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_x = Tkinter.Label(self.leap_window, text='X:').grid(row=9, column=0) 
    self.leap_offset_label_xv = Tkinter.Label(self.leap_window, textvariable=self.leap_offset_x).grid(row=9, column=1) 
    self.leap_offset_y = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_y = Tkinter.Label(self.leap_window, text='Y:').grid(row=10, column=0) 
    self.leap_offset_label_yv = Tkinter.Label(self.leap_window, textvariable=self.leap_offset_y).grid(row=10, column=1) 
    self.leap_offset_z = Tkinter.StringVar(self.leap_window, "0.00 ")
    self.leap_offset_label_z = Tkinter.Label(self.leap_window, text='Z:').grid(row=11, column=0) 
    self.leap_offset_label_zv = Tkinter.Label(self.leap_window, textvariable=self.leap_offset_z).grid(row=11, column=1) 
    

    #input scale
    self.gui_scale = Tkinter.StringVar(self.leap_window, self.space_modifier)
    self.leap_label_scalev = Tkinter.Entry(self.leap_window, textvariable=self.gui_scale).grid(row=13, column=1)

    #movement tolerance
    self.gui_tolerance = Tkinter.StringVar(self.leap_window, self.space_modifier)
    self.leap_label_tolerancev = Tkinter.Entry(self.leap_window, textvariable=self.gui_tolerance).grid(row=14, column=1)

    #Add buttons for resetting the offset variable, and setting the panda arm back to its initial position
    self.reset_button = Tkinter.Button(self.leap_window, text = "Reset Offset", command = self.reset_off).grid(row=12,column=1)
    self.reset_button = Tkinter.Button(self.leap_window, text = "Reset Robot", command = self.reset_pos).grid(row=15,column=1)

    #set the scale and tolerance based off of the values assigned in the init function
    self.gui_scale.set(self.space_modifier)
    self.gui_tolerance.set(self.tolerance)

  #Used to update the GUI with new leap data
  def updateGui(self):
    #left hand
    self.gui_left_x.set(self.doubleToString(self.leap.left_location[0]))
    self.gui_left_y.set(self.doubleToString(self.leap.left_location[1]))
    self.gui_left_z.set(self.doubleToString(self.leap.left_location[2]))
    self.gui_left_roll.set(self.doubleToString(self.leap.left_orientation[0]))
    self.gui_left_pitch.set(self.doubleToString(self.leap.left_orientation[1]))
    self.gui_left_yaw.set(self.doubleToString(self.leap.left_orientation[2]))
    self.gui_left_grab.set(self.doubleToString(self.leap.grab))

    #right hand
    self.gui_right_x.set(self.doubleToString(self.leap.right_location[0]))
    self.gui_right_y.set(self.doubleToString(self.leap.right_location[1]))
    self.gui_right_z.set(self.doubleToString(self.leap.right_location[2]))
    self.gui_right_roll.set(self.doubleToString(self.leap.right_orientation[0]))
    self.gui_right_pitch.set(self.doubleToString(self.leap.right_orientation[1]))
    self.gui_right_yaw.set(self.doubleToString(self.leap.right_orientation[2]))
    self.gui_right_pinch.set(self.doubleToString(self.leap.pinch))

    #offset
    self.leap_offset_x .set(self.offset_x)
    self.leap_offset_y.set(self.offset_y) 
    self.leap_offset_z.set(self.offset_z)
    
    #space modifier and tolerance is taken as an input during this call and assigned to the class variables
    self.space_modifier = float(self.gui_scale.get())
    self.tolerance = float(self.gui_tolerance.get())
  
  #used to quickly convert all double values to a string with 2dp
  def doubleToString(self, input):
    return round(input, 2)

  #code to convert roll pitch yaw into quaternion and perform transformations to make it compatible with the panda arm
  def Quaternion(self, pose, roll, pitch, yaw):

    #tr quarternion accepts pose orientation x,y,z,w format

    if(self.orientation_mode):      
      #get the quaternion from rpy
      tmp2 = tr.euler.euler2quat(roll, pitch, yaw)

      #create a rotation matrix
      rm = np.array([[1, 0, 0],[0, 0, 1],[0, -1, 0]])
      
      #convert the martrix into a quaternion
      tmp = tr.quaternions.mat2quat(rm)

      #multiply
      out = tr.quaternions.qmult(tmp2, tmp)
      
      #set orientation to pose
      pose.orientation = geometry_msgs.msg.Quaternion(*out)
      
      #tell user
      print "Experimental Pose: ", pose.orientation

    else:
      #make the orientation the same as the default pose
      pose.orientation = self.default_pose.pose.orientation
      
      #tell user
      print "Pose: ", pose.orientation

    return pose
  
  #callback function from leap to panda
  def callback(self, data):
    #store the new leap data
    self.leap = data
    #update the gui based on this new data
    self.updateGui()

    #if we dont have the default pose, get it, this is also treated as a first run handler and captures the calibration distance
    if(self.default_pose == None):
      self.default_pose = self.arm.get_current_pose("panda_link8")
      self.calibration = self.leap.calibration
      #if calibration isnt 0, half it
      if self.calibration != 0:
        self.calibration = self.calibration/2

    #if we are waiting to send a command to the panda arm
    if self.waiting:
      #we are not waiting anymore
      self.waiting = False
      
      #if the left hand is detected check to see if we are moving the workspace
      if(abs(self.leap.left_location[0])!=abs(self.calibration) or self.leap.left_location[1] !=0 or self.leap.left_location[2] != 0):
        self.move_workspace()

      #if the right hand is detected control the arm
      if(abs(self.leap.right_location[0])!=abs(self.calibration) or self.leap.right_location[1] != 0 or self.leap.right_location[2]!= 0):
        self.go_to_pinch_goal()
        self.go_to_pose_goal()
      
      #we are waiting for a new command
      self.waiting = True

  #used to reset the position of the panda arm
  def reset_pos(self):
    #we dont need a new command
    self.waiting = False
    print("Resetting robot")
    
    #get the current pose
    current_arm_pose = self.arm.get_current_pose("panda_link8")
    
    #set the target to be the previously stored default pose
    self.arm.set_pose_target(self.default_pose, "panda_link8")

    #move
    self.arm.go(wait=True)
    self.arm.stop()

    #we are now waiting again
    self.waiting = True
    
    #all close
    return all_close(self.default_pose, current_arm_pose, self.tolerance)

  #reset the offset variables
  def reset_off(self):
    #we dont need a new command
    self.waiting = False

    print("Resetting offset")

    #reset the variables
    self.offset_x = 0
    self.offset_y = 0
    self.offset_z = 0
    self.previous_offset = [0,0,0]
    
    #we are now waiting again
    self.waiting = True

  #Code for moving the workspace using our left hand
  def move_workspace(self):
    #if the grab gesture isnt strong enough return
    if(self.leap.grab > 0.9):

      #update the offsets based on the current position and previous offset
      self.offset_x = self.offset_x + (self.previous_offset[0] - self.leap.left_location[0])
      self.offset_y = self.offset_y + (-1*(self.previous_offset[1] - self.leap.left_location[1]))
      self.offset_z = self.offset_z + (self.previous_offset[2] - self.leap.left_location[2])

      #tell the user
      print ("")
      print ("Moving workspace: grip = ", self.leap.grab)
      print ("From:", self.previous_offset[0], self.previous_offset[1], self.previous_offset[2])
      print ("To:", self.leap.left_location[0], self.leap.left_location[1], self.leap.left_location[2])
      print ("")

      #update the previous offset
      self.previous_offset = self.leap.left_location

  #Code for moving the panda arm
  def go_to_pose_goal(self):
    #get the current pose
    current_arm_pose = self.arm.get_current_pose("panda_link8").pose
    #set up a new pose object and fill it with the position and orientation
    arm_pose_goal = geometry_msgs.msg.Pose()
    arm_pose_goal = self.Quaternion(arm_pose_goal,self.leap.right_orientation[0],self.leap.right_orientation[1],self.leap.right_orientation[2]) #gets the quaternion based on rpy
    
    #get the pos using the current ( hand location + the offset ) * the scale
    arm_pose_goal.position.x = ((self.leap.right_location[0])+self.offset_x)*self.space_modifier
    arm_pose_goal.position.z = ((self.leap.right_location[1])+self.offset_y)*self.space_modifier #y and z are flipped
    arm_pose_goal.position.y = -1*((self.leap.right_location[2])+self.offset_z)*self.space_modifier # y is inverted

    #tell the user
    print ("")
    print ("Moving to:")
    print (arm_pose_goal.position.x, arm_pose_goal.position.y, arm_pose_goal.position.z)
    print (arm_pose_goal.orientation.w, arm_pose_goal.orientation.x, arm_pose_goal.orientation.y, arm_pose_goal.orientation.z)
    print ("")

    #set the target to be the pose we just created
    self.arm.set_pose_target(arm_pose_goal, "panda_link8")
    
    #move
    self.arm.go(wait=True)
    self.arm.stop()
    self.arm.clear_pose_targets()
    
    #all close
    return all_close(arm_pose_goal, current_arm_pose, self.tolerance)
  
  #code to operate the pandas grabber
  def go_to_pinch_goal(self):    
    #get the current joint values for the all close
    current_hand_joints = self.arm.get_current_joint_values()
    
    #tell the user the current pinch strength
    print ("")
    print ("Pinching:", self.leap.pinch)

    #get the current joint values to edit for the new position
    hand_joint_goal = self.arm.get_current_joint_values()

    #set the grabber joints manually
    #0.035 is the joint limit and leap.pinch is a value from 0 to 1, 0 being closed, one being open
    hand_joint_goal[7] = 0.035 - (self.leap.pinch*0.035)
    hand_joint_goal[8] = 0.035 - (self.leap.pinch*0.035)

    #move the grabber
    self.arm.go(hand_joint_goal, wait=True)
    self.arm.stop()

    #all close
    return all_close(hand_joint_goal, current_hand_joints, self.tolerance)

#main function
def main():
  #main try catch loop
  try:
    #create a new object
    test = LeapMoveGroup()
    print (">----------Closed----------<")
    print (">------Any key to exit-----<")
    raw_input()

  except:
    return

#run main
if __name__ == '__main__':
  main()