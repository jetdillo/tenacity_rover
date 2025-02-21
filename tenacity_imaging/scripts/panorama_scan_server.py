#!/usr/bin/env python

import rospy
import actionlib
import os
import uuid
from datetime import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
from tenacity_imaging.msg import PanoramaScanAction, PanoramaScanResult

class PanoramaScanActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('panorama_scan', PanoramaScanAction, self.execute, False)
        self.server.start()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rgb_stereo_publisher/color/image', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/rgb_stereo_publisher/stereo/depth', Image, self.depth_callback)
        self.pan_pub = rospy.Publisher('/mastcam_pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/mastcam_tilt_controller/command', Float64, queue_size=10)
        self.current_image = None
        self.current_depth = None
        self.image_ready = False
        self.depth_ready = False

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_ready = True

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_ready = True

    def execute(self, goal):
        # Create a directory for the scan
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #scan_dir = f"/tmp/tenacity_scan_{timestamp}"
        scan_dir = "/tmp/tenacity_scan_"+timestamp
        #os.makedirs(scan_dir, exist_ok=True)
        if not os.path.exists(scan_dir):
            os.makedirs(scan_dir)

        # Initialize tilt position
        tilt_angle = 0.0
        tilt_increment = 30.0
        tilt_direction = 1  # 1 for positive, -1 for negative

        while True:
            # Set tilt position
            self.tilt_pub.publish(Float64(tilt_angle))
            rospy.sleep(2)  # Wait for the tilt servo to reach the position

            # Initialize pan position
            pan_angle = 90.0
            self.pan_pub.publish(Float64(pan_angle))
            rospy.sleep(2)  # Wait for the pan servo to reach the initial position

            while pan_angle >= -90.0:
                # Capture image and depth
                self.image_ready = False
                self.depth_ready = False
                while not self.image_ready or not self.depth_ready:
                    rospy.sleep(0.1)

                # Save image and depth
                image_filename = scan_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".jpg"

                depth_filename = scan_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".png"
                cv2.imwrite(image_filename, self.current_image)
                cv2.imwrite(depth_filename, self.current_depth)

                # Move to the next pan position
                pan_angle -= 30.0
                self.pan_pub.publish(Float64(pan_angle))
                rospy.sleep(2)  # Wait for the servo to reach the next position

            # Wait 5 seconds at -90 degrees pan
            rospy.sleep(5)

            # Return pan to 0 degrees
            self.pan_pub.publish(Float64(0.0))
            rospy.sleep(2)

            # Update tilt angle
            if tilt_direction == 1:
                tilt_angle += tilt_increment
                if tilt_angle >= 90.0:
                    tilt_direction = -1
                    tilt_angle = -30.0  # Start moving in the negative direction
            else:
                tilt_angle -= tilt_increment
                if tilt_angle < -90.0:
                    break  # Exit the loop when tilt reaches -90 degrees

        # Return the result
        result = PanoramaScanResult()
        result.scan_directory = scan_dir
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('panorama_scan_server')
    server = PanoramaScanActionServer()
    rospy.spin()
