#!/usr/bin/env python

import rospy
import math
#from OpticalSteeringCorrection.srv import OpticalSteeringCorrection, OpticalSteeringCorrectionResponse
from openmv_flow.srv import OpticalSteeringCorrection, OpticalSteeringCorrectionResponse

def optical_steering_correction(pixel_drift_x, pixel_drift_y, time_elapsed):
    """
    Calculate steering corrections based on pixel drift.
    
    Args:
        pixel_drift_x (float): Drift in the x-direction (horizontal) in pixels.
        pixel_drift_y (float): Drift in the y-direction (vertical) in pixels.
        time_elapsed (float): Time elapsed since the last measurement (seconds).
        
    Returns:
        float: Steering correction in radians.
    """
    # Retrieve parameters from the ROS Parameter Server
    camera_height = rospy.get_param('camera_height')
    camera_tilt = rospy.get_param('camera_tilt')
    horizontal_fov = rospy.get_param('horizontal_fov')
    vertical_fov = rospy.get_param('vertical_fov')
    image_width = rospy.get_param('image_width')
    image_height = rospy.get_param('image_height')

    # Convert tilt angle to radians
    tilt_radians = math.radians(camera_tilt)

    # Calculate ground distance to the center of the image
    ground_distance = camera_height * math.tan(tilt_radians)

    # Calculate horizontal and vertical scales (mm/pixel)
    horizontal_scale = (2 * ground_distance * math.tan(math.radians(horizontal_fov / 2))) / image_width
    vertical_scale = (2 * ground_distance * math.tan(math.radians(vertical_fov / 2))) / image_height

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

def handle_optical_steering_correction(req):
    """
    Service handler for calculating steering correction.
    """
    rospy.loginfo("Received request: pixel_drift_x=%f, pixel_drift_y=%f, time_elapsed=%f", req.pixel_drift_x, req.pixel_drift_y, req.time_elapsed)
    steering_correction = optical_steering_correction(req.pixel_drift_x, req.pixel_drift_y, req.time_elapsed)
    rospy.loginfo("Calculated steering correction: %f radians", steering_correction)
    return OpticalSteeringCorrectionResponse(steering_correction)

def optical_flow_service_server():
    """
    Initialize the ROS node and start the service server.
    """
    rospy.init_node('optical_flow_service_server')

    # Retrieve parameters from the ROS Parameter Server
    rospy.loginfo("Loading parameters from the ROS Parameter Server...")
    openmv_flow_params=rospy.get_param('/openmv_flow')
    camera_name = rospy.get_param('camera_name', 'openmv_cam')
    rospy.loginfo("Camera Name: %s", camera_name)
    for p in openmv_flow_params:
        rospy.loginfo(p)

    # Start the service
    s = rospy.Service('optical_steering_correction', OpticalSteeringCorrection, handle_optical_steering_correction)
    rospy.loginfo("Optical Steering Correction Server is ready.")
    rospy.spin()

if __name__ == "__main__":
    optical_flow_service_server()
