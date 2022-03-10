#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
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
#  * Neither the name of Robotiq, Inc. nor the names of its
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
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$ 
# samplePoints 0,1,2,3,4,5 0,100,50,150,150,0

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from time import sleep
import math
from math import sin, cos, pi
import time
import numpy as np
import sys
from PIDController import PIDController
import csv
import threading
import _thread as thread

class Robotiq2FGripperSlipController(object):
    def __init__(self):
        self.feedback_signal = inputMsg.Robotiq2FGripper_robot_input();
        self.meas_force = 0
        self.ref_force = 0
        self.run_force_control = False
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.run_slip_control = False
        self.slipping = False
        self.slip_speed = 0
        self.new_force = False


    def data(self, data_from_sub):
        #global feedback_signal
        self.feedback_signal = data_from_sub
        
    def get_force(self, data_from_sub):
        #global meas_force
        self.meas_force = data_from_sub.data

    def get_ref_force(self, data_from_sub):
        #global ref_force
        self.ref_force =  data_from_sub.data
        
    def get_slip(self, data_from_sub):
        #global slipping
        self.slipping = data_from_sub.data
        
    def get_slip_speed(self, data_from_sub):
        #global slip_speed
        self.slip_speed = data_from_sub.data
    
    def update_list_slip(self, list_slip, new_val):
        new_list = []
        for i in range(len(list_slip)-1):
            new_list.append(list_slip[i+1])
            
        new_list.append(new_val)
        return new_list

    def force_control(self):
        #global command
        #global new_force
        
        control_force = PIDController(0,0.6,0.1,0.006)
        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size = 1)
        rate = rospy.Rate(30)
        """
        while True:
            
            rospy.Subscriber("force", Float64, get_force, queue_size = 1)
            rospy.Subscriber("ref_force", Float64, get_ref_force, queue_size = 1)
            rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,data, queue_size = 1)

            #Admittance force control (from paper)
            force_error = ref_force - meas_force
            pos_error = control_force.control(force_error,0)

            new_pos = feedback_signal.gPO + pos_error
            #print(new_pos)
            command = doCommand('GoTo',int(new_pos),0,255,command)
            pub.publish(command)
            rate.sleep()

            if not run_force_control:
                break
        """
        while self.run_force_control:
            rospy.Subscriber("estimatedForce", Float64, self.get_force, queue_size=1)
            rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)
            
            if new_force:
                control_force.clear()
                new_force = False
                
            force_error = ref_force - self.meas_force
            pos_error = control_force.control(force_error,0)
            #print(pos_error)
            
            new_pos = self.feedback_signal.gPO + int(pos_error)
            command = doCommand('GoTo',int(new_pos),255,255,command)
            pub.publish(command)
            
            rate.sleep()
            
        print("Disable force control")
        return
        


    def slip_control_force(self):
        #global command
        #global new_force
        
        control_slip = PIDController(0,0.2,0,0)
        #control_force = PIDController(0,1,0.1,0.005)
        #control_force = PIDController(0,1,0.1,0.0)
        control_force = PIDController(0,0.6,0.1,0.006)
        
        
        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size = 1)
        rate = rospy.Rate(30)
        ever_slipping = False
        list_SLIP = [False]*3
        while run_slip_control:
            #print("Slip control")
            rospy.Subscriber("SLIP", Bool, self.get_slip, queue_size = 1)
            rospy.Subscriber("SLIP_speed",Float64,self.get_slip_speed,queue_size=1)
            rospy.Subscriber("estimatedForce",Float64,self.get_force,queue_size=1)
            rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)
            
            list_SLIP = self.update_list_slip(list_SLIP, self.slipping)
            
            if (not ever_slipping) and all(list_SLIP):
                ever_slipping = True
                print("SLIP")
            
            if self.slipping:
                #print("SLIP")
                print(self.slip_speed)
                pos_error = control_slip.control(self.slip_speed,0)
                pos_new = self.feedback_signal.gPO+math.ceil(pos_error)
                command = doCommand("GoTo",pos_new,255,255,command)
                
                #print(math.ceil(pos_error))
                
            elif new_force:
                force_error = ref_force - self.meas_force
                control_force.clear()
                pos_error = control_force.control(force_error,0)
                new_pos = self.feedback_signal.gPO + round(pos_error)
                command = doCommand('GoTo',int(new_pos),255,255,command)
                new_force = False
                ever_slipping = False
                
            elif (not ever_slipping):
                force_error = ref_force - self.meas_force
                #print(force_error)
                pos_error = control_force.control(force_error,0)
                print(str(pos_error)+"-->"+str(round(pos_error)))
                new_pos = self.feedback_signal.gPO + round(pos_error)
                command = doCommand('GoTo',int(new_pos),255,255,command)
            else:
                new_pos = self.feedback_signal.gPO
                command = doCommand('GoTo',int(new_pos),255,255,command)
            
            pub.publish(command)
            rate.sleep()
            
        print("Disabled slip control")
        return

    def slip_control_pos(self):
        #global command
        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size = 1)
        
        rate = rospy.Rate(30)
        while run_slip_control:
            #print("Slip control")
            rospy.Subscriber("SLIP", Bool, self.get_slip, queue_size = 1)
            if self.slipping:
                print("SLIP")
                rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)
                pos_new = self.feedback_signal.gPO+1
                command = doCommand("GoTo",pos_new,0,255,command)
                pub.publish(command)
                while self.feedback_signal.gPO != pos_new:
                    rate.sleep()
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)
            rate.sleep()
            
        print("Disabled slip control")
        return

    def disable_slip_control(self):
        #global run_slip_control
        #global ref_force
        #global new_force
        #rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            inputtext = "Enter slip control value\n"
            inputtext += "Enter \'E\' to exit slip control\n"
            text = input(inputtext)
            if text == 'E':
                self.run_slip_control = False
                return
            else:
                try:
                    text = float(text)
                    self.ref_force = text
                    self.new_force = True
                    print("ref_force = "+str(self.ref_force))
                except:
                    print("Input not valid")
        self.run_slip_control = False
        return

    def disable_force_control(self):
        #global run_force_control
        #global ref_force
        #global new_force
        #rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            inputtext = "Enter force control value\n"
            inputtext += "Enter \'E\' to exit force control\n"
            text = input(inputtext)
            if text == 'E':
                self.run_force_control = False
                return
            else:
                try:
                    text = float(text)
                    self.ref_force = text
                    self.new_force = True
                    print("ref_force = "+str(self.ref_force))
                except:
                    print("Input not valid")
        self.run_force_control = False
        return
    

    def genPath(self, input_string, f_sample):
        """Update the command according to the character entered by the user."""    

        inputs = input_string.split(' ')

        if len(inputs) == 1:
            char = inputs[0]

            if char == 'a':
                pathType = 'a'

            elif char == 'r':
                pathType = 'r'

            elif char == 'c':
                pathType = 'c'

            elif char == 'o':
                pathType = 'o'

            elif char == 'g':
                pathType = 'g'

            elif char == 'fg':
                pathType = 'fg'

            elif char == 'sfg':
                pathType = 'sfg'

            elif char == 'rg':
                pathType = 'rg'

            elif char == 'trap_g':
                pathType = 'trap_g'

            else:
                sys.exit()

            return pathType, 1/f_sample, None, None

        elif len(inputs) == 2 and inputs[0] == 'f':
            pathType = 'f'
            return pathType, 1/f_sample, inputs[1], None

        elif len(inputs) == 2 and inputs[0] == 'fg':
            pathType = 'fg'
            return pathType, 1/f_sample, inputs[1], None

        elif len(inputs) == 2 and inputs[0] == 'sfg':
            pathType = 'sfg'
            return pathType, 1/f_sample, inputs[1], None

        elif len(inputs) == 2 and inputs[0] == 'sq':
            pathType = 'sq'
            return pathType, 1/f_sample, inputs[1], None
        
        elif len(inputs) == 2 and inputs[0] == 'fc':
            pathType = 'fc'
            return pathType, 1/f_sample, inputs[1], None
        
        elif len(inputs) == 2 and inputs[0] == 'sc':
            pathType = 'sc'
            return pathType, 1/f_sample, inputs[1], None
        
        elif len(inputs) == 4 and inputs[0] == 'cal':
            pathType = 'cal'
            return pathType, 1/f_sample, inputs, None

        elif len(inputs) == 4 and inputs[0] == 'GoTo':
            pathType = 'GoTo'
            return pathType, 1/f_sample, inputs, None

        elif len(inputs) == 2 and inputs[0] == 'GoTo':
            pathType = 'GoTo'
            return pathType, 1/f_sample, inputs, None

        elif len(inputs) == 4 and inputs[0] == 'sin':
            pathType = 'sin'	
            a = float(inputs[1])
            T = float(inputs[2])
            duration = float(inputs[3])
            f_sample = f_sample

            t_sample = []
            pos_sample = []
            speed_sample = []
            
            global feedback_signal
            pos_zero = self.feedback_signal.gPO

            #pos_old = command.rPr #old position command
            for t in range(int(duration*f_sample)):
                t_sample.append(t/f_sample)
                pos = a/2*(1+sin(2*pi/T*t/f_sample-pi/2)) + pos_zero #bit
                speed = a/2*cos(2*pi/T*t/f_sample-pi/2)*(2*pi/T) #bit/s
                speed = -47.497+0.6836*abs(speed)

                if pos < 0:
                    pos = 0
                elif pos > 255:
                    pos = 255

                if speed < 0:
                    speed = 0
                elif speed > 255:
                    speed = 255

                pos_sample.append(int(pos))
                speed_sample.append(int(speed))

            #print(speed_sample)
            return pathType, t_sample, pos_sample, speed_sample


        elif len(inputs) == 4 and inputs[0] == 'tri':
            pathType = 'tri'	
            a = float(inputs[1])
            T = float(inputs[2])
            duration = float(inputs[3])
            f_sample = f_sample

            t_sample = []
            pos_sample = []
            speed_sample = []

            mm = a/255*85.75
            
            #global feedback_signal
            pos_zero = self.feedback_signal.gPO

            if round(duration/T,1) % 1 != 0:
                print("duration and period not compatible")
                sys.exit()

            #pos_old = command.rPr #old position command
            for t in range(int(duration/T*2)+1):
                t_sample.append(t*T/2)
                
                if t % 2 == 0:
                    pos = a + pos_zero
                else:
                    pos = pos_zero

                if pos < 0:
                    pos = 0
                elif pos > 255:
                    pos = 255

                pos_sample.append(int(pos))
                # new linearisation (MATLAB)
                dt = T/2-0.0645
                v = a/dt
                speed = -47.497+0.6836*v
                #v = mm/(T/2-0.0645)
                #speed = (v - 20.96)/(141.06-20.96)*255
                #speed = int(speed) 
                #m = 0.21847580077769402 #experimentally determined relationship
                #b = 10.358194489115007 #between speed setting and real velocity
                #speed = int((v-b)/(2*m))
                if speed < 0:
                    speed = 0
                elif speed > 255:
                    speed = 255
                
                speed_sample.append(speed)

            print("speed = ",speed)            
            return pathType, t_sample, pos_sample, speed_sample

        elif len(inputs) == 5 and inputs[0] == 'trap':
            pathType = 'trap'
            h = float(inputs[1])
            bs = float(inputs[2])
            BL = float(inputs[3])
            duration = float(inputs[4])

            t_sample = []
            pos_sample = []
            speed_sample = []
            
            pos_zero = self.feedback_signal.gPO
            
            t = 0
            while t < duration:
                p = 0
                while p < BL:
                    t = round(t*f_sample)/f_sample
                    t_sample.append(t)
                    if p < (BL - bs)/2:
                        v = h/((BL-bs)/2)
                        m = 1.1455792557267448 #experimentally determined relationship
                        b = 54.45956174537362 #between speed setting and real velocity
                        speed = int((v-b)/m)
                        if speed < 0:
                            speed = 0
                        elif speed > 255:
                            speed = 255

                        pos = p*v + pos_zero
                        if pos < 0:
                            pos = 0
                        elif pos > 255:
                            pos = 255
                        pos_sample.append(int(pos))
                        speed_sample.append(speed)
                        p += 1/f_sample
                        t += 1/f_sample
                
                    elif p < bs + (BL-bs)/2:
                        pos = h + pos_zero
                        if pos < 0:
                            pos = 0
                        elif pos > 255:
                            pos = 255

                        speed = 0
                        pos_sample.append(int(pos))
                        speed_sample.append(speed)
                        p += 1/f_sample
                        t += 1/f_sample
                    else:
                        v = h/((BL-bs)/2)
                        m = 1.1455792557267448 #experimentally determined relationship
                        b = 54.45956174537362 #between speed setting and real velocity
                        speed = int((v-b)/m)
                        if speed < 0:
                            speed = 0
                        elif speed > 255:
                            speed = 255

                        pos = h - v*(p - bs - (BL - bs)/2) + pos_zero
                        if pos < 0:
                            pos = 0
                        elif pos > 255:
                            pos = 255
                        pos_sample.append(int(pos))
                        speed_sample.append(speed)
                        p += 1/f_sample
                        t += 1/f_sample
        
            return pathType, t_sample, pos_sample, speed_sample

        elif len(inputs) == 3 and inputs[0] == 'samplePoints':
            pathType = 'samplePoints'	
            t_list = inputs[1].split(',')
            pos_list = inputs[2].split(',')
            
            t_list = [int(t) for t in t_list]
            pos_list = [int(pos) for pos in pos_list]

            if t_list[0] != 0:
                sys.exit('List should start at t = 0s')

            if len(t_list) != len(pos_list):
                sys.exit('Lists should have the same length')

            t_sample = []
            pos_sample = []
            speed_sample = []

            for i in range(len(t_list)-1):
                t = t_list[i]
                t = round(t*f_sample)/f_sample
                t_sample.append(t)
                pos = pos_list[i]
                if pos < 0:
                    pos = 0
                elif pos > 255:
                    pos = 255
                pos_sample.append(pos)
                pos_start = pos
                t_start = t

                pos_end = pos_list[i+1]
                if pos_end < 0:
                    pos_end = 0
                elif pos_end > 255:
                    pos_end = 255

                t_end = t_list[i+1]
                t_end = round(t_end*f_sample)/f_sample

                m = 1.1455792557267448 #experimentally determined relationship
                b = 54.45956174537362 #between speed setting and real velocity

                speed = abs(pos_end - pos_start)/(t_end - t_start)
                speed = int((speed-b)/m)
                if speed < 0:
                    speed = 0
                elif speed > 255:
                    speed = 255
                speed_sample.append(speed)
                t += 1/f_sample
                while t < t_end:
                    t = round(t*f_sample)/f_sample
                    t_sample.append(t)
                    pos = pos_start + (pos_end - pos_start)/(t_end - t_start)*(t - t_start)
                    pos = int(pos)
                    if pos < 0:
                        pos = 0
                    elif pos > 255:
                        pos = 255
                    pos_sample.append(pos)
                    speed_sample.append(speed)
                    t += 1/f_sample

            return pathType, t_sample, pos_sample, speed_sample

        else:
            sys.exit()

        
    def doCommand(self, pathType, pos, speed, force, command):
        if pathType == 'a':
            self.command = outputMsg.Robotiq2FGripper_robot_output();
            self.command.rACT = 1
            self.command.rGTO = 1
            self.command.rSP  = 255
            self.command.rFR  = 150
            return self.command

        elif pathType == 'r':
            self.command = outputMsg.Robotiq2FGripper_robot_output();
            self.command.rACT = 0
            return self.command

        elif pathType == 'c':
            self.command.rGTO = 1
            self.command.rPR = 255
            self.command.rSP = 255
            self.command.rFR = 255
            return self.command

        elif pathType == 'o':
            self.command.rGTO = 1
            self.command.rPR = 0
            self.command.rSP = 255
            self.command.rFR = 255
            return self.command

        elif pathType == 'g':
            self.command.rGTO = 1
            self.command.rPR = 255
            self.command.rSP = 0
            self.command.rFR = 0
            return self.command
    
        elif pathType == 'fg':
            self.command.rGTO = 1
            self.command.rPR = 255
            self.command.rSP = 0
            self.command.rFR = 255
            """
            try: 
                    command.rFR = int(force)
                    if command.rFR > 255:
                        command.rFR = 255
                    if command.rFR < 0:
                        command.rFR = 0
            except ValueError:
                    pass
            """
            return self.command
        

        elif pathType == "Stop":
            self.command.rGTO = 0
            return self.command

        elif pathType == 'GoTo':
            self.command.rGTO = 1
            try: 
                self.command.rPR = int(pos)
                if self.command.rPR > 255:
                    self.command.rPR = 255
                if self.command.rPR < 0:
                    self.command.rPR = 0
            except ValueError:
                pass                    
            
            try: 
                self.command.rSP = int(speed)
                if self.command.rSP > 255:
                    self.command.rSP = 255
                if self.command.rSP < 0:
                    self.command.rSP = 0
            except ValueError:
                pass  

            try: 
                self.command.rFR = int(force)
                if self.command.rFR > 255:
                    self.command.rFR = 255
                if self.command.rFR < 0:
                    self.command.rFR = 0
            except ValueError:
                pass
            return self.command

    """
        elif pathType == 'sin':
            speed_sample = 127*ones(len(t_sample), dtype=int)

            control = PIDController(f_sample,8,16,1)

            time_start = time.time()
            while time.time() - time_start < t_sample[-1]:
                time_now = time.time()
                t_delta = round((time_now-time_start)*f_sample)/f_sample
                index = t_sample.index(delta_t)
                pos_set = pos_sample[index]
                feedback = rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
                pos_current = feedback.gPO

                if feedback.gPR > pos_current:
                    opening = False
                else:
                    opening = True

                control_out = control.control(pos_set,pos_current)

                if opening:
                    speed_set = int(speed_sample[i] - control_out)
                else:
                    speed_set = int(speed_sample[i] + control_out)

                if speed_set < 0:
                    speed_set = 0
                elif speed_set > 255:
                    speed_set = 255

                command.rPR = pos_set
                command.rSP = speed_set
                command.rFR = 0
                pub.publish(command)
                rospy.sleep(1/f_sample)
                return
    """
    """
        elif pathType == 'trap':


        elif pathType == 'samplePoints':
    """


    def askForInput(self, command):
        """Ask the user for a command to send to the gripper."""    

        currentCommand  = 'Advanced 2F Gripper Controller\n-----\nCurrent command:'
        currentCommand += '  rACT = '  + str(command.rACT)
        currentCommand += ', rGTO = '  + str(command.rGTO)
        currentCommand += ', rATR = '  + str(command.rATR)
        currentCommand += ', rPR = '   + str(command.rPR )
        currentCommand += ', rSP = '   + str(command.rSP )
        currentCommand += ', rFR = '   + str(command.rFR )


        print(currentCommand)

        strAskForCommand  = '-----\nAvailable commands\n\n'
        strAskForCommand += 'r: Reset\n'
        strAskForCommand += 'a: Activate\n'
        strAskForCommand += 'cal obit cbit width: Calibrate return 1 bit in mm\n'
        strAskForCommand += 'c: Close (full speed & force)\n'
        strAskForCommand += 'o: Open (full speed & force)\n'
        strAskForCommand += 'g: Grap (minimal speed & force)\n'
        strAskForCommand += 'GoTo pos speed force\n'
        strAskForCommand += 'f x: Set force during trajectory to x\n'
        strAskForCommand += 'sin amplitude period duration\n'
        strAskForCommand += 'trap height small_b large_b duration\n'
        strAskForCommand += 'tri amplitude period duration\n'
        strAskForCommand += 'samplePoints t_list pos_list\n'
        strAskForCommand += 'fg: Grap with force detection (minimal speed & 1 N force)\n'
        strAskForCommand += 'fg x: Grap with x Newton (minimal speed)\n'
        strAskForCommand += 'sfg: Grap with force detection (minimal speed & 1 N force)\n'
        strAskForCommand += 'sfg x: Grap with x Newton (minimal speed)\n'
        strAskForCommand += 'sq x: Squeeze x positions\n'
        strAskForCommand += 'fc x: Force control @ x N\n'
        #strAskForCommand += 'rg: Regrasp sequence cherry tomato\n'
        #strAskForCommand += 'trap_g: Ramp up as stair - cherry tomato\n'
        strAskForCommand += 'sc x: Slip Control @ x N\n'

        strAskForCommand += '-->'

        return input(strAskForCommand)

    def controller(self):
        """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
        #global run_force_control
        #global command
        #global ref_force

        sample_frequency = 200 #Hz
        force_setting = 0
        obit = 0
        cbit = 255
        width = 85 #mm

        rospy.init_node('Robotiq2FGripperForceController')
        rate = rospy.Rate(sample_frequency)

        pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size = 1)

        #command = outputMsg.Robotiq2FGripper_robot_output();
        
        control = PIDController(0,10,10,0.02)
        control_sin = PIDController(0,1,0,0)
        
        while not rospy.is_shutdown():

            inputs = self.askForInput(self.command)
            """
            if run_force_control:
                run_force_control = False
                thread_fc.join()
                print("Force control terminated")
            """
            notYetPrinted = True

            rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)

            pathType, t_sample, pos_sample, speed_sample = self.genPath(inputs, sample_frequency)

            opening = False #first always closing

            if speed_sample == None: 
                try:
                    speed_sample = 127*np.ones(len(t_sample), dtype=int)
                except:
                    pass

            t_start = time.time()

            try:
                t_end = t_sample[-1]
            except:
                t_end = t_sample

            while time.time() - t_start < t_end:

                if pathType == 'trap' or pathType == 'samplePoints': #or pathType == 'sin':
                    time_now = time.time()
                    t_delta = round((time_now-t_start)*sample_frequency)/sample_frequency
                    index = t_sample.index(t_delta)
                    pos_set = pos_sample[index]
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)
                    pos_current = self.feedback_signal.gPO

                    opening = False #first always closing
                    try:                
                        if pos_sample[index] < pos_sample[index-1] and opening == False:
                            opening = True
                        elif pos_sample[index] > pos_sample[index-1] and opening == True:
                            opening = False
                    except: #first always closing
                        opening = False

                    control_out = control.control(pos_set,pos_current)
                    #print('pos:',pos_current,', pos_set:',pos_set,', opening:',opening)
                    #control_out = 0

                    if opening:
                        speed_set = int(speed_sample[index] + control_out)
                    else:
                        speed_set = int(speed_sample[index] - control_out)
                    

                    if speed_set < 0:
                        speed_set = 0
                    elif speed_set > 255:
                        speed_set = 255
                    
                    #print('control_out:',control_out, 'speed:',speed_set)
                    if notYetPrinted:
                        print('force_setting = ',force_setting)
                        notYetPrinted = False
                    command = self.doCommand('GoTo',pos_set, speed_set,force_setting, command)
                    pub.publish(command)
                    rate.sleep()
                    
                elif pathType == 'sin':
                    time_now = time.time()
                    t_delta = round((time_now-t_start)*sample_frequency)/sample_frequency
                    index = t_sample.index(t_delta)
                    pos_set = pos_sample[index]
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,self.data, queue_size = 1)
                    pos_current = self.feedback_signal.gPO

                    try:                
                        if pos_sample[index] < pos_sample[index-1] and opening == False:
                            opening = True
                        elif pos_sample[index] > pos_sample[index-1] and opening == True:
                            opening = False
                    except: 
                        pass #opening = False -- already ealier defined

                    control_out_sin = control_sin.control(pos_set,pos_current)
                    #print('pos:',pos_current,', pos_set:',pos_set,', opening:',opening)
                    #control_out = 0

                    if opening:
                        speed_set = int(speed_sample[index] + control_out_sin)
                    else:
                        speed_set = int(speed_sample[index] - control_out_sin)
                    
                    #print(speed_set)

                    if speed_set < 0:
                        speed_set = 0
                    elif speed_set > 255:
                        speed_set = 255
                    
                    #print('control_out:',control_out, 'speed:',speed_set)
                    if notYetPrinted:
                        print('force_setting = ',force_setting)
                        notYetPrinted = False
                    
                    if opening:
                        pos_extr = min(pos_sample)
                    else:
                        pos_extr = max(pos_sample)

                    command = self.doCommand('GoTo',pos_extr, speed_set,force_setting, command)
                    pub.publish(command)
                    rate.sleep()

                elif pathType == 'tri':
                    time_now = time.time()                
                    try:
                        T = t_sample[2]-t_sample[0]
                    except:
                        try:
                            T = 2*(t_sample[2]-t_sample[0])
                        except:
                            print("duration too short")
                            sys.exit()

                    index = int((time_now-t_start)/(T/2))
                    command = self.doCommand('GoTo',pos_sample[index],speed_sample[index],force_setting,command)
                    pub.publish(command)
                    
                    time.sleep(T/2)

                elif pathType == 'f':
                    force_setting = int(pos_sample)
                    if force_setting < 0:
                        force_setting = 0
                    elif force_setting > 255:
                        force_setting = 255

                    print('force_setting = ',force_setting)
                    rate.sleep()
                    break
                
                elif pathType == 'cal':
                    obit = float(pos_sample[1])
                    cbit = float(pos_sample[2])
                    width = float(pos_sample[3])

                    print('1 bit = ',width/(cbit-obit),' mm')
                    rate.sleep()
                    break
                
                elif pathType == 'fg':
                    pos = 255
                    speed = 0
                    if pos_sample == None:
                        max_force = 1
                    else:
                        max_force = float(pos_sample)

                    # set up tolerance to better approximate wanted force and not only get overshoot
                    tolerance = 0.05
                    max_force = (1-tolerance)*max_force
                    
                    command = self.doCommand(pathType, pos, speed, force, command)
                    pub.publish(command)
                    rate.sleep()
                    
                    rospy.Subscriber("estimatedForce", Float64, self.get_force, queue_size = 1)
                    
                    while self.meas_force < max_force:
                        rospy.Subscriber("estimatedForce", Float64, self.get_force, queue_size = 1)
                        #rate.sleep()
                        time.sleep(0.01)

                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.data, queue_size = 1)
                    #command = doCommand("GoTo",feedback_signal.gPO+1,0,255,command)
                    command = self.doCommand("Stop", self.feedback_signal.gPO+1,0,255,command)
                    pub.publish(command)
                    rate.sleep()
                        
                    break

                elif pathType == 'sfg':
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,data, queue_size = 1)
                    pos_0 = self.feedback_signal.gPO
                    n = 255 - pos_0

                    if pos_sample == None:
                        max_force = 1
                    else:
                        max_force = float(pos_sample)

                    # set up tolerance to better approximate wanted force and not only get overshoot
                    tolerance = 0.10
                    max_force = (1-tolerance)*max_force

                    for i in range(n):
                        force_array = np.array([])
                        pos_0 = pos_0+1
                        command = self.doCommand("GoTo",pos_0,0,255,command)
                        pub.publish(command)
                        for j in range(20):
                            rospy.Subscriber("estimatedForce", Float64, self.get_force, queue_size = 1)
                            force_array = np.append(force_array, self.meas_force)
                            time.sleep(0.01)
                        print(np.average(force_array))
                        if np.average(force_array) > max_force:
                            break
                        else:
                            time.sleep(0.5)

                    
                    rate.sleep()
                    break

                elif pathType == 'sq':
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,data, queue_size = 1)
                    pos_new = self.feedback_signal.gPO+int(pos_sample)
                    command = self.doCommand("GoTo",pos_new,0,255,command)
                    pub.publish(command)
                    rate.sleep()
                        
                    break

                elif pathType == 'rg':
                    # first squeuze up to +- 7.5 N
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,data, queue_size = 1)
                    pos_new = self.feedback_signal.gPO+13
                    command = self.doCommand("GoTo",pos_new,0,255,command)
                    pub.publish(command)
                    rate.sleep()
                    # then +1/-1 every 1/2/? seconds for x times
                    for i in range(5):
                        pos_new = pos_new - 1
                        command = self.doCommand("GoTo",pos_new,0,255,command)
                        pub.publish(command)
                        time.sleep(1)
                        pos_new = pos_new + 1
                        command = self.doCommand("GoTo",pos_new,0,255,command)
                        pub.publish(command)
                        time.sleep(1)
                    
                    break

                elif pathType == 'trap_g':
                    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,data, queue_size = 1)
                    pos_new = self.feedback_signal.gPO
                    for i in range(13):
                        pos_new = pos_new + 1
                        command = self.doCommand("GoTo",pos_new,0,255,command)
                        pub.publish(command)
                        time.sleep(0.5)

                    time.sleep(5)

                    for i in range(13):
                        pos_new = pos_new - 1
                        command = self.doCommand("GoTo",pos_new,0,255,command)
                        pub.publish(command)
                        time.sleep(0.5)

                    break
                    
                elif pathType == 'fc':
                    global run_force_control
                    run_force_control = True
                    
                    ref_force = float(pos_sample)
                    
                    thread.start_new_thread(self.force_control,())
                    thread.start_new_thread(self.disable_force_control,())
                    
                    while (not rospy.is_shutdown()) and run_force_control:
                        rate.sleep()
                        pass
                    
                    time.sleep(0.1)
                    break
                
                elif pathType == 'sc':
                    global run_slip_control
                    run_slip_control = True
                    
                    ref_force = float(pos_sample)
                    
                    thread.start_new_thread(self.slip_control_force,())
                    #thread.start_new_thread(slip_control_pos,())
                    thread.start_new_thread(self.disable_slip_control,())
                    
                    while (not rospy.is_shutdown()) and run_slip_control:
                        rate.sleep()
                        pass
                    
                    time.sleep(0.1)
                    break

                else:
                    try:
                        pos = pos_sample[1]
                    except:
                        pos = 0

                    try:
                        speed = pos_sample[2]
                        force = pos_sample[3]
                    except:
                        speed = 0
                        force = 0

                    command = self.doCommand(pathType, pos, speed, force, command)
                    pub.publish(command)
                    rate.sleep()
                    break
            


def main(args=None):
    rospy.init_node("Robotiq2FGripperSlipController", anonymous=True)
    my_node = Robotiq2FGripperSlipController()
    my_node.controller()

if __name__ == '__main__':
    main()