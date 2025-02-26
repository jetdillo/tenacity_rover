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
from tenacity_imaging.msg import MastCamSnapshotAction, MastCamSnapshotResult
from dynamixel_msgs.msg import JointState as JointState_DM


class MastCamSnapshotActionServer:
    def __init__(self):
        #self.server = actionlib.SimpleActionServer('mastcam_snapshot', MastCamSnapshotAction, self.execute, False)
        self.server = actionlib.SimpleActionServer(rospy.get_name(),MastCamSnapshotAction,execute_cb=self.execute_cb,auto_start=False)
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

        self.key_positions=rospy.get_param("/mastcam_snapshot_server/diag_points")

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

    def execute_cb(self, goal):
        # Create a directory for the snapshot

        rospy.loginfo('%s action server taking snapshot at %s' % (rospy.get_name(),goal.target_name))

        pan_pos,tilt_pos=self.keypoints(goal.target_name)

        self.tilt_pub.publish(Float64(tilt_angle))
        self.pan_pub.publish(Float64(pan_angle))

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #snapshot_dir = f"/tmp/tenacity_snapshot_{timestamp}"
        snapshot_dir = "/tmp/tenacity_snapshot_"+timestamp
        rgb_dir = snapshot_dir+"/rgb"
        depth_dir = snapshot_dir+"/depth"
        #os.makedirs(snapshot_dir, exist_ok=True)
        if not os.path.exists(snapshot_dir):
            os.makedirs(rgb_dir)
            os.makedirs(depth_dir)
   
        # Initialize tilt position
        tilt_angle = 0.0

        # Initialize pan position 
        pan_angle = 0.0

        rospy.loginfo("Taking snapshot at commanded position")
        rospy.loginfo("pan_angle:%f tilt_angle:%f",self.pan_pos,self.tilt_pos)

        while True:
            # Set tilt position
            pan_angle = (90.0 * math.pi) / 180 
            self.tilt_pub.publish(Float64(tilt_angle))
            self.pan_pub.publish(Float64(pan_angle))
            rospy.sleep(3)  # Wait for the tilt servo to reach the position

            # Initialize pan position
            # Capture image and depth
            self.image_ready = False
            self.depth_ready = False
            while not self.image_ready or not self.depth_ready:
                rospy.sleep(1)
            rospy.loginfo("image_ready:%s depth_ready: %s",self.image_ready,self.depth_ready)
                    
            rospy.loginfo("Grabbing image at camera position %f %f",self.pan_pos,self.tilt_pos)
                    # Save image and depth
            #image_filename = rgb_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".jpg"
            image_filename = rgb_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+str(self.pan_pos)+"_"+str(self.tilt_pos)+".jpg"
    
            #depth_filename = snapshot_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".png"
            depth_filename = depth_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(self.pan_pos)+"_"+str(self.tilt_pos)+".png"
            cv2.imwrite(image_filename, self.current_image)
            cv2.imwrite(depth_filename, self.current_depth)
    
            # Wait 5 seconds at -90 degrees pan
            rospy.sleep(5)

            break  # Exit the loop when tilt reaches -90 degrees

        # Return the result
        result = MastCamSnapshotResult()
        result.snapshot_directory = snapshot_dir
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('mastcam_snapshot_server')
    server = MastCamSnapshotActionServer()
    rospy.spin()
