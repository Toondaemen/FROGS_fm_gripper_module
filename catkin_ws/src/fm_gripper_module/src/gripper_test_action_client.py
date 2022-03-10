#! /usr/bin/env python
import rospy
import actionlib
from fm_gripper_msgs.msg import FmGripperAction, FmGripperGoal

def gripper_action_client():

  # Cast the data in the right format
  force = 1
  deformable = False
  reset = False
  activate = False
  calibrate = False
  open = True
  close = False
  force_control = False
  slip_control = False

  # Establish connection with UR action server
  client = actionlib.SimpleActionClient('gripper_action_server', FmGripperAction)
  client.wait_for_server()

 
  # Send the trajectory data over the action server
  goal = FmGripperGoal()
  goal.force = force
  goal.deformable = deformable
  goal.command_reset = reset
  goal.command_activate = reset
  goal.command_calibrate = calibrate
  goal.command_open = open
  goal.command_close = close
  goal.command_force_control = force_control
  goal.command_slip_control = slip_control

  client.send_goal(goal)
  client.wait_for_result()

  return client.get_result()


if __name__ == '__main__':
  rospy.init_node('gripper_action_client')
  result = gripper_action_client()

