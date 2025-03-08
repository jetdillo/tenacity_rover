#!/usr/bin/env python

import math
import rospy
import actionlib
import os
import uuid
import yaml
from datetime import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
from tenacity_imaging.msg import MastCamSnapshotAction, MastCamSnapshotFeedback,MastCamSnapshotResult
from dynamixel_msgs.msg import JointState as JointState_DM


class MastCamSnapshotActionServer:
    def __init__(self):
        self.mcss = actionlib.SimpleActionServer(rospy.get_name(),
                      MastCamSnapshotAction,
                      execute_cb=self.execute_cb,
                      auto_start=False)
        self.mcss.start()
        rospy.loginfo("MastCam Action Server %s Started" % rospy.get_name())

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

        with open('/home/ubuntu/catkin_ws/src/tenacity_imaging/config/mastcam_snapshot_params.yaml', 'r') as file:
            params = yaml.safe_load(file)
            for key, value in params.items():
                rospy.set_param(key, value)
                rospy.loginfo("%s = %s" % (key,value))

        self.vistas=rospy.get_param("/key_points")

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
        success=False
        rospy.loginfo('%s action mcss taking snapshot at %s' % (rospy.get_name(),goal.vista_name))
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

        rospy.loginfo("Homing mastcam")
        self.tilt_pub.publish(Float64(tilt_angle))
        self.pan_pub.publish(Float64(pan_angle))
       
        rospy.sleep(3)

        feedback = MastCamSnapshotFeedback()
        feedback.fb = "Homed to mastcam pan:"+str(self.pan_pos)+" mastcam tilt:"+str(self.tilt_pos)
        self.mcss.publish_feedback(feedback)
   
        rospy.loginfo("Moving mastcam to commanded position")

            # Set tilt position
        pan_target,tilt_target=self.vistas[goal.vista_name]
        self.tilt_pub.publish(Float64(tilt_target))
        self.pan_pub.publish(Float64(pan_target))

        rospy.sleep(3)

        feedback = MastCamSnapshotFeedback()
        feedback.fb = "Reached mastcam pan:"+str(self.pan_pos)+" mastcam tilt:"+str(self.tilt_pos)
        self.mcss.publish_feedback(feedback)

        rospy.loginfo("Taking snapshot at %s",goal.vista_name)

            # Capture image and depth
        self.image_ready = False
        self.depth_ready = False
        while not self.image_ready or not self.depth_ready:
            rospy.sleep(1)
            feedback = MastCamSnapshotFeedback()
            feedback.fb = "Waiting on camera stream..."
            self.mcss.publish_feedback(feedback)

        rospy.loginfo("image_ready:%s depth_ready: %s",self.image_ready,self.depth_ready)
                    
        image_filename = rgb_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+str(self.pan_pos)+"_"+str(self.tilt_pos)+".jpg"
    
            #depth_filename = snapshot_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(uuid.uuid4())+".png"
        depth_filename = depth_dir+"/"+datetime.now().strftime('%Y%m%d_%H%M%S')+"_"+str(self.pan_pos)+"_"+str(self.tilt_pos)+".png"
        try:
           cv2.imwrite(image_filename, self.current_image)
           cv2.imwrite(depth_filename, self.current_depth)
           success=True
        except cv2.error as e: 
           success=False 
           rospy.loginfo("Snapshot Action failed with error %s",e)
           result=self.get_default_result()
           abort_txt=e
           self.mcss.set_aborted(result,abort_txt)

        # Return the result
        result = MastCamSnapshotResult()
        result.img_path = snapshot_dir
        self.mcss.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('mastcam_snapshot_server')
    mcss = MastCamSnapshotActionServer()
    rospy.spin()
