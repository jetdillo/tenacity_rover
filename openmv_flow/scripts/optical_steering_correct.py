#!/usr/bin/env python2.7

# Python libraries
import sys, os
import serial
import time
import argparse
import math
import rospy
from std_msgs.msg import Float32
from openmv_flow.msg import OpticalDrift,OpticalDriftCorrection

# Import OpenCV
import cv2 as cv
import numpy as np

# Constants
#CAMERA_HEIGHT = 250  # mm
#CAMERA_TILT = 30  # degrees
#HORIZONTAL_FOV = 60.7  # degrees
#VERTICAL_FOV = 47.5  # degrees
#image_width = 128  # pixels
#image_height = 128  # pixels
#ROBOT_SPEED = 250  # mm/s (0.25 m/s)

cam_height=rospy.get_param("/openmv_flow/camera_height")
camera_tilt=rospy.get_param("/openmv_flow/camera_tilt")
hfov=rospy.get_param("/openmv_flow/horizontal_fov")
vfov=rospy.get_param("/openmv_flow/vertical_fov")
image_width=rospy.get_param("/openmv_flow/image_width")
image_height=rospy.get_param("/openmv_flow/image_height")
robot_speed=rospy.get_param("/openmv_flow/robot_speed")

def handle_correction(pixel_drift_x, pixel_drift_y, time_elapsed):
    # Convert tilt angle to radians
    tilt_radians = math.radians(camera_tilt)
    # Calculate ground distance to the center of the image
    ground_distance = camera_height * math.tan(tilt_radians)
    # Calculate horizontal and vertical scales (mm/pixel)
    horizontal_scale = (2 * ground_distance * math.tan(math.radians(horizontal_fov / 2))) / image_width
    # Convert pixel drift to millimeters
    drift_x_mm = pixel_drift_x * horizontal_scale
    drift_y_mm = pixel_drift_y * vertical_scale
    # Calculate drift rate (mm/s)
    drift_rate_x = drift_x_mm / time_elapsed
    drift_rate_y = drift_y_mm / time_elapsed

    # Proportional control gain (adjust based on robot behavior)
    Kp = 0.01  # Proportional gain

    # Calculate steering correction (radians)
    steering_correction = -Kp * drift_rate_x  # Negative sign to counteract drift

    return steering_correction

def steering_correction_server():
    #Load in a bunch of camera parameters from the camera server

    rospy.init_node('optical_steering_correction_server')
    svc = rospy.Service('optical_steering_correct',OpticalDriftCorrection,handle_correction)
    print("Drift Correction Server ready...")
    rospy.spin()

# Example usage
if __name__ == "__main__":

    steering_correction_server()
