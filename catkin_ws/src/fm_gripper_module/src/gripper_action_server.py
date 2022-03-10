#! /usr/bin/env python

import rospy
import actionlib
from actionlib.msg import TestAction, TestFeedback, TestResult

from fm_gripper_msgs.msg import FmGripperAction, FmGripperResult, FmGripperFeedback

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input  as inputMsg
from time import sleep
from Robotiq2FGripperSlipController import Robotiq2FGripperSlipController

class GripperActionServer(object):


    def __init__(self):

        # Init server
        rospy.init_node('fm_gripper_action_server')
        self.server = actionlib.SimpleActionServer('gripper_action_server', FmGripperAction, execute_cb = self.execute, auto_start = False)
        self.rate = rospy.Rate(10.0)

        self.feedback = FmGripperFeedback()
        self.result = FmGripperResult()
        self.server.start()
    

    def execute(self, goal):
        self.state = 'initializing'
        rospy.loginfo('state: ' + self.state)
          
        if goal.command_reset == True:
            command = 'r'
        elif goal.command_activate == True:
            command = 'a'
        elif goal.command_calibrate == True:
            command = 'cal'
        elif goal.command_open == True:
            command = 'o'
        elif goal.command_close == True:
            command = 'c'
            if goal.closure:
                command = 'GoTo ' + goal.closure
        elif goal.command_force_control == True:
            force = goal.force
            command = 'fc ' + str(force)
        elif goal.command_slip_control == True:
            force = goal.force
            command = 'sc ' + str(force)


        while (not self.server.is_preempt_requested()) and (not rospy.is_shutdown()):
            Robotiq2FGripperSlipController().controller()                  
            self.result.result = 1 
            self.server.set_succeeded(self.result)
            self.state = 'inactive'
            rospy.loginfo('state: ' + self.state)
            return 

        self.server.set_preempted()
        self.state = 'inactive'
        rospy.loginfo('state: ' + self.state)

if __name__ == '__main__':
    server = GripperActionServer()
    rospy.spin()
