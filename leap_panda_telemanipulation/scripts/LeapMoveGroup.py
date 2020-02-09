#!/usr/bin/env python
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from leap_panda_telemanipulation.msg import Modified_leap
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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
    pose.orientation.x = cy * cp * sr - sy * sp * cr
    pose.orientation.y = sy * cp * sr + cy * sp * cr
    pose.orientation.z = sy * cp * cr - cy * sp * sr

    return pose

class LeapMoveGroup(object):

  def __init__(self):
    super(LeapMoveGroup, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('LeapMoveGroup', anonymous=True)


    robot = moveit_commander.RobotCommander()

    arm = moveit_commander.MoveGroupCommander("panda_arm")
    #hand = moveit_commander.MoveGroupCommander("panda_hand")

    # We can also print the name of the end-effector link for this arm:
    eef_link = arm.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.robot = robot
    self.arm = arm
    self.eef_link = eef_link
    self.group_names = group_names
    
    #take in Modified_leap.msg
    self.leap = Modified_leap()

    self.listener()

  def callback(self, data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    self.leap = data

  def listener(self):
    rospy.Subscriber("leap_to_panda", Modified_leap, self.callback)
    rospy.spin()

  def go_to_pose_goal(self):
    ## Planning to a Pose Goal
    ## We can plan a motion for this arm to a desired pose for the
    ## end-effectors:
    
    arm_pose_goal = geometry_msgs.msg.Pose()
    arm_pose_goal = Quaternion(arm_pose_goal,self.leap.right_orientation[0],self.leap.right_orientation[1],self.leap.right_orientation[2])
    arm_pose_goal.position.x = self.leap.right_location[0]
    arm_pose_goal.position.y = self.leap.right_location[1]
    arm_pose_goal.position.z = self.leap.right_location[2]


    print "moving to:"
    print arm_pose_goal

    self.arm.set_pose_target(arm_pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    arm_plan = self.arm.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.arm.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.arm.clear_pose_targets()

    current_arm_pose = self.arm.get_current_pose().pose

    return all_close(arm_pose_goal, current_arm_pose, 0.01)

def main():
  try:
    test = LeapMoveGroup()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    test.go_to_pose_goal()

    print "============ Python demo complete! Press `Enter` to complete"
    raw_input()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
