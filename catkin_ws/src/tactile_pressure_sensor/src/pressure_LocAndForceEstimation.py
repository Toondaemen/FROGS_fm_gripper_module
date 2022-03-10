#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  9 11:37:08 2021

@author: pc-robotiq

Updated:
    - by: Merwan BIREM on Tuesday 16 November 12:44
        -->> Converted the script to ROS package
"""
import rospy
import time
import math
import sys
import os
import numpy as np

from std_msgs.msg import Float64MultiArray, Bool
from pressure_pkg.utils import HerzianContactLoc, butter_lowpass_filter, SlipDetectionDist

# import message 
from tactile_pressure_sensor.msg import PressureData


class LocAndForceEstimation(object):
    def __init__(self):
        # variables and parameters
        freq = 30
    
        global t
        global t0
        
        # Configuration of the pressure sensor array
        self.k = 0.001272164923549179 # from CalibrationForce_All.py
        self.shape = [3,5]
        self.spacing = 9 # mm
        self.pressure_data      = np.zeros((1,15))
        self.pressure_data_init = np.zeros((1,15))

        # set internal variables        
        self.rate = rospy.Rate(freq)
        self.threshold_pressure = 2500
        self.success = False
        self.calibrated = False
        self.coord_xy = Float64MultiArray()
        self.initialization = False
        self.pressure_reset = False

        self.x_sample = []
        self.y_sample = []
        self.x_sample_fund = []
        self.y_sample_fund = []

        for i in range(freq*1):
            self.x_sample.append(0)
            self.y_sample.append(0)

        # Subscribers
        rospy.Subscriber('/sensor/pressure_reset', Bool, self.pressure_reset_sub, queue_size=1)
        rospy.Subscriber('/sensor/pressure_data' , Float64MultiArray, self.pressure_data_sub, queue_size=1)

        # Publishers
        self.estimated_ForceAndLoc = PressureData()
        self.pub_tactilePressure = rospy.Publisher('estimated_ForceAndLoc', PressureData, queue_size=1)

        time.sleep(3.0)

        pass


    def pressure_reset_sub(self, msg):
        if msg.data == True:
            self.set_pressure_init()
            self.calibrated = True
            self.rate.sleep()


    def pressure_data_sub(self, msg):
        for i in range(15):
            self.pressure_data[0][i] = msg.data[i]
        
        if not(self.initialization):
            self.set_pressure_init()
            self.calibrated = True
            self.rate.sleep()
            self.initialization = True

    def set_pressure_init(self):    
        for i in range(self.pressure_data_init.shape[1]):
            self.pressure_data_init[0][i] = self.pressure_data[0][i]


    def start(self):
        rospy.loginfo("pressure data processing package started ")

        self.estimated_ForceAndLoc.sensor_state    = "idle"
        self.estimated_ForceAndLoc.slip_status     = False                
        self.estimated_ForceAndLoc.slip_speed      = 0.0
        self.estimated_ForceAndLoc.slip_angle      = 0.0
        self.estimated_ForceAndLoc.estimated_force = 0.0
        self.estimated_ForceAndLoc.estimated_location.data = [0.0, 0.0]
        
        
        while not rospy.is_shutdown() and self.initialization:
            t0 = time.time()
            # delta pressure between current and previous
            pressures = []
            for i in range(self.pressure_data.shape[1]):
                pressures.append(self.pressure_data[0][i]-self.pressure_data_init[0][i])

            rospy.loginfo("diff between pressures val = {} ".format(pressures))
            rospy.loginfo("--> current  pressure val = {} ".format(self.pressure_data))
            rospy.loginfo("--> previous pressure val = {} ".format(self.pressure_data_init))

            # NOTE increase this value to make it less sensitive to pressure
            if np.max(pressures) > self.threshold_pressure:
                arrayResults, E = HerzianContactLoc(pressures, self.shape, self.spacing)                        
                self.success    = E.success
                resultEx        = E.x

                if self.success:
                    if (len(self.x_sample_fund) < len(self.x_sample)) and (len(self.y_sample_fund) < len(self.y_sample)):
                        self.x_sample_fund.append(resultEx[2])
                        self.y_sample_fund.append(resultEx[3])
                        
                        n = math.floor(len(self.x_sample)/len(self.x_sample_fund))
                        for idx in range(len(self.x_sample)):
                            idxx = math.floor(idx/n)
                            if idxx >= len(self.x_sample_fund):
                                idxx = len(self.x_sample_fund)-1
                            self.x_sample[idx] = self.x_sample_fund[idxx]
                            self.y_sample[idx] = self.y_sample_fund[idxx]
                    else:
                        _ = self.x_sample_fund.pop(0)
                        self.x_sample_fund.append(resultEx[2])
                        _ = self.y_sample_fund.pop(0)
                        self.y_sample_fund.append(resultEx[3])
                        
                        self.x_sample = self.x_sample_fund.copy()
                        self.y_sample = self.y_sample_fund.copy()

                    x_f = butter_lowpass_filter(self.x_sample,False)
                    y_f = butter_lowpass_filter(self.y_sample,False)
                    
                    self.x_sample[-1] = x_f[-1]
                    self.y_sample[-1] = y_f[-1]
                    self.x_sample_fund[-1] = x_f[-1]
                    self.y_sample_fund[-1] = y_f[-1]
                    
                    slip_status, slip_speed, slip_angle = SlipDetectionDist(self.x_sample, self.y_sample, 
                                                                    frac=0.2, 
                                                                    dist_thres=1.2,
                                                                    use_last=15)

                    estimated_force = self.k*resultEx[0]*resultEx[1]                    
                    self.coord_xy.data = [x_f[-1], y_f[-1]]                            
                    self.calibrated = False

                    self.estimated_ForceAndLoc.sensor_state    = "success"
                    self.estimated_ForceAndLoc.slip_status     = slip_status              
                    self.estimated_ForceAndLoc.slip_speed      = slip_speed
                    self.estimated_ForceAndLoc.slip_angle      = slip_angle
                    self.estimated_ForceAndLoc.estimated_force = estimated_force
                    self.estimated_ForceAndLoc.estimated_location = self.coord_xy


                else:
                    pass

                    self.estimated_ForceAndLoc.sensor_state    = "no-success"
                    self.estimated_ForceAndLoc.slip_status     = False                
                    self.estimated_ForceAndLoc.slip_speed      = 0.0
                    self.estimated_ForceAndLoc.slip_angle      = 0.0
                    self.estimated_ForceAndLoc.estimated_force = 0.0
                    self.estimated_ForceAndLoc.estimated_location.data = [0.0, 0.0]

            else:
                ###  
                self.success = False
                ###  re-initialize the pressure sensor with last values
                self.set_pressure_init()
                self.calibrated = True
                ### 
                for i in range(len(self.x_sample)):
                    _ = self.x_sample.pop(0)
                    self.x_sample.append(0)
                    _ = self.y_sample.pop(0)
                    self.y_sample.append(0)
                    
                    self.x_sample_fund = []
                    self.y_sample_fund = []
                ###                 
                self.estimated_ForceAndLoc.sensor_state    = "faillure"
                self.estimated_ForceAndLoc.slip_status     = False                
                self.estimated_ForceAndLoc.slip_speed      = 0.0
                self.estimated_ForceAndLoc.slip_angle      = 0.0
                self.estimated_ForceAndLoc.estimated_force = 0.0
                self.estimated_ForceAndLoc.estimated_location.data = [0.0, 0.0]
                

            # Publish the final results !!! 
            self.pub_tactilePressure.publish(self.estimated_ForceAndLoc)
                
            self.rate.sleep()
        



def main(args=None):
    rospy.init_node("PressureLocAndForceEstimation", anonymous=True)
    my_node = LocAndForceEstimation()
    my_node.start()

if __name__ == '__main__':
    main()