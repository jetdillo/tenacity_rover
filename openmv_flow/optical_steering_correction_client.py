#!/usr/bin/env python

import rospy
import math
import sys, os
import time
import argparse
from std_msgs.msg import Float32
from openmv_flow.srv import OpticalSteeringCorrection, OpticalSteeringCorrectionResponse
from openmv_flow.msg import OpticalDrift

pixel_drift_x=0.0
pixel_drift_y=0.0

steering_d = {'pixel_drift_x':0.0,'pixel_drift_y':0.0,'correction':0.0}

#What we really should be doing is collecting drift over a time interval
#and issuing a correction for a drift beyond a certain amount over interval I 
#This lets you be robust to bumps and shakes and running over certain obstacles
#in a short amount of time

def optical_drift_cb(data):
    pixel_drift_x = data.xdiff
    pixel_drift_y = data.ydiff

def steering_correction_runner():
    rospy.Subscriber("/optical_drift",OpticalDrift,optical_drift_cb)
    rospy.wait_for_service('optical_steering_correction')
    try:
        get_correction = rospy.ServiceProxy('optical_steering_correction',OpticalSteeringCorrection)
        turn = get_correction(pixel_drift_x,pixel_drift_y,1.0)
        steering_d.pixel_drift_x=pixel_drift_x
        steering_d.pixel_drift_y=pixel_drift_y
        steering_d.correction=turn
        return steering_d 
    except:
        rospy.loginfo("Error connecting to OpticalSteeringCorrection service");
        sys.exit(1)
       
if __name__ == "__main__":

    sd=steering_correction_runner() 
    print("x_drift:%f y_drift:%f correction:%f",sd.pixel_drift_x,sd.pixel_drift_y,sd.correction)
