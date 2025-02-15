#!/usr/bin/env python2.7

# Python libraries
import sys, os
import serial
import time
import argparse

import rospy
from std_msgs.msg import Float32
from openmv_flow.msg import OpticalDrift

# Import OpenCV
import cv2 as cv
import numpy as np

# Main
if __name__ == '__main__':

    port = '/dev/openmvcam'
    try:
       serial_port = serial.Serial(port, baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
   
       serial_port.flush()
    except serial.SerialException as e:

       print("COULD NOT OPEN SERIAL PORT %s" % port)
       sys.exit()

    rospy.init_node("optical_flow")
    odpub = rospy.Publisher("optical_drift",OpticalDrift,queue_size=10)
    rospy.loginfo("Started OpenMV Optical Drift Tracker...")
    buf = serial_port.readline()
    rospy.loginfo("%s",buf)
    #We should really be publishing on the Pi, using the existing course trimmer
    while not rospy.is_shutdown():
        # Read data from the serial buffer
        buf = serial_port.readline()
        #rospy.loginfo("%s",buf)
        x_diff,y_diff,response,fps=[float(v) for v in buf.split()]

        od = OpticalDrift()
        od.xdiff = x_diff
        od.ydiff = y_diff
        od.response = response

        odpub.publish(od)
