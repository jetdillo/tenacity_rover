#!/usr/bin/env python

import math
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
from dynamixel_msgs.msg import JointState as JointState_DM


class PanoramaScanActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('panorama_scan', PanoramaScanAction, self.execute, False)
        self.server.start()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rgb_stereo_publisher/color/image', Image, self.image_cb)
        self.depth_sub = rospy.Subscriber('/rgb_stereo_publisher/stereo/depth', Image, self.depth_cb)
        self.pan_sub = rospy.Subscriber('/mastcam_pan_controller/state', JointState_DM, self.pan_cb)
        self.tilt_sub = rospy.Subscriber('/mastcam_tilt_controller/state', JointState_DM, self.tilt_cb)
        self.pan_pub = rospy.Publisher('/mastcam_pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/mastcam_tilt_controller/command', Float64, queue_size=10)
        self.current_image = None
        self.current_depth = None
        self.image_ready = False
        self.depth_ready = False

        self.pan_pos = 0.0
        self.tilt_pos = 0.0

    def image_cb(self, img):
        self.current_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        self.image_ready = True

    def depth_cb(self, img):
        self.current_depth = self.bridge.imgmsg_to_cv2(img, "passthrough")
        self.depth_ready = True

    def pan_cb(self, data):
        self.pan_pos=data.current_pos

    def tilt_cb(self, data):
        self.tilt_pos=data.current_pos

    def execute(self, goal):
        # Create a directory for the scan
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #scan_dir = f"/tmp/tenacity_scan_{timestamp}"
        scan_dir = "/tmp/tenacity_scan_"+timestamp
        rgb_dir = scan_dir+"/rgb"
        depth_dir = scan_dir+"/depth"
        #os.makedirs(scan_dir, exist_ok=True)
        if not os.path.exists(scan_dir):
            os.makedirs(rgb_dir)
            os.makedirs(depth_dir)

        # Initialize tilt position
        tilt_angle = 0.0
        tilt_increment = rospy.get_param("/panorama_scan_server/tilt_increment")
        tilt_direction = 1  # 1 for positive, -1 for negative
 
        tilt_points = rospy.get_param("/panorama_scan_server/tilt_points")

        # Initialize pan position 
        pan_angle = 0.0
        pan_increment = rospy.get_param("/panorama_scan_server/pan_increment")
        pan_direction = 1

        rospy.loginfo("Starting panorama capture")
        rospy.loginfo("pan_angle:%f tilt_angle:%f",self.pan_pos,self.tilt_pos)

        while True:
            # Set tilt position
            self.tilt_pub.publish(Float64(tilt_angle))
            rospy.sleep(3)  # Wait for the tilt servo to reach the position

            # Initialize pan position
            pan_angle = (90.0 * math.pi) / 180 
            self.pan_pub.publish(Float64(pan_angle))
            rospy.sleep(3)  # Wait for the pan servo to reach the initial position

            for tilt_angle in tilt_points:
                self.tilt_pub.publish(Float64(tilt_angle))
                pan_angle = (90.0 * math.pi) / 180 
                self.pan_pub.publish(Float64(pan_angle))
                rospy.sleep(3)  # Wait for the pan servo to reach the initial position
                
                while pan_angle >= -1.57:
                    rospy.loginfo("PAN ACTION - MOVING TO pan_angle:%f tilt_angle:%f",self.pan_pos,self.tilt_pos)
                    # Capture image and depth
                    self.image_ready = False
                    self.depth_ready = False
                    while not self.image_ready or not self.depth_ready:
                        rospy.sleep(1)
                        rospy.loginfo("image_ready:%s depth_ready: %s",self.image_ready,self.depth_ready)
                    
                    rospy.loginfo("DUMPING FRAMES")
                    # Save image and depth
                    #image_filename = rgb_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".jpg"
                    image_filename = rgb_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+str(self.pan_pos)+"_"+str(self.tilt_pos)+".jpg"
    
                    #depth_filename = scan_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".png"
                    depth_filename = depth_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(self.pan_pos)+"_"+str(self.tilt_pos)+".png"
                    cv2.imwrite(image_filename, self.current_image)
                    cv2.imwrite(depth_filename, self.current_depth)
    
                    # Move to the next pan position
                    pan_angle -= pan_increment
                    self.pan_pub.publish(Float64(pan_angle))
                    rospy.sleep(5)  # Wait for the servo to reach the next position
    
            # Wait 5 seconds at -90 degrees pan
                rospy.sleep(5)

            break  # Exit the loop when tilt reaches -90 degrees

        # Return the result
        result = PanoramaScanResult()
        result.scan_directory = scan_dir
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('panorama_scan_server')
    server = PanoramaScanActionServer()
    rospy.spin()
