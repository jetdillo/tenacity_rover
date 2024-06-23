#!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2
import numpy as np
import sys

from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import String,Float64
from geometry_msgs.msg import Point,Twist
from sensor_msgs.msg import Image
from dynamixel_msgs.msg import JointState

from vision_msgs.msg import ObjectHypothesis,BoundingBox2D
from depthai_ros_msgs.msg import SpatialDetection
from depthai_ros_msgs.msg import SpatialDetectionArray

class yoloFollower:

   def __init__(self):
   
      rospy.init_node("yolo_detector")

      self.yolo_str="/yolov4_publisher/color/yolov4_Spatial_detections"
      self.yolo_depth_str="/yolov4_publisher/stereo/depth"
      self.yolo_obj_topic="/yolov4/detected_objects"

      self.drive=Twist()
      self.curpos=Twist()

      self.steer_state=JointState()

      self.yolo_frame_dims=[]
      self.yolo_frame_roi_keypt=0
      self.yolo_frame_roi_last_keypt=0
      self.yolo_frame_roi_last_depth=0
      self.yolo_rgb="/yolov4_publisher/color/image"
      self.yolo_rgb_out="/yolov4_publisher/color/annotated_image"
      self.yolo_img=Image()
      self.yolo_depth=Image()
      self.sda=SpatialDetectionArray()
      self.detected_object=SpatialDetection()
      self.cvb=CvBridge()
     
      rospy.Subscriber(self.yolo_str,SpatialDetectionArray, self.sda_proc_cb)
      rospy.Subscriber(self.yolo_rgb,Image, self.yolo_img_cb)
      rospy.Subscriber(self.yolo_depth_str,Image,self.yolo_depth_cb)
      rospy.Subscriber('/front_left_controller/state',JointState,self.steering_state_cb)
      self.boxed_image=rospy.Publisher(self.yolo_rgb_out,Image,queue_size=1)

      self.drive=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
      self.front_left_sp=rospy.Publisher('/front_left_controller/command',Float64,queue_size=1)
      self.front_right_sp=rospy.Publisher('/front_right_controller/command',Float64,queue_size=1)
      self.rear_left_sp=rospy.Publisher('/rear_left_controller/command',Float64,queue_size=1)
      self.rear_right_sp=rospy.Publisher('/rear_right_controller/command',Float64,queue_size=1)

   def sda_proc_cb(self,data):
      self.sda=data

   def yolo_img_cb(self,data):
      self.yolo_img=data 

   def yolo_depth_cb(self,data):
      self.yolo_depth=self.cvb.imgmsg_to_cv2(data)
      self.yolo_frame_dims=np.shape(self.yolo_depth)

   def steering_state_cb(self,data):
       js=JointState()
       js.current_pos=data.current_pos
       self.steer_state=js



   def roi_turn(self,front_newpos,rear_newpos):
      self.front_left_sp.publish(front_newpos) 
      self.front_right_sp.publish(front_newpos) 

      self.rear_left_sp.publish(rear_newpos) 
      self.rear_right_sp.publish(rear_newpos) 

   def follow_roi(self,roi_x,roi_y,roi_depth):

      roi_front_js = JointState()
      roi_rear_js = JointState()

      roi_shift=roi_x - self.yolo_frame_roi_last_keypt
      roi_move=roi_depth - self.yolo_frame_roi_last_depth

      if (abs(roi_shift) >10) :
          if roi_shift >1:
             #turn left 
             rospy.loginfo("FOLLOW_ROI: LEFT TURN with roi_shift of %d",roi_shift)
             front_newpos=self.steer_state.current_pos+0.17
             rear_newpos=front_newpos*-1

          if roi_shift <1:
             rospy.loginfo("FOLLOW_ROI: RIGHT TURN with roi_shift of %d",roi_shift)
             front_newpos=self.steer_state.current_pos-0.17
             rear_newpos=front_newpos*-1
             
          self.roi_turn(front_newpos,rear_newpos)
      else:
          self.roi_turn(self.steer_state.current_pos,self.steer_state.current_pos*-1) 

      self.yolo_frame_roi_last_keypt=roi_x
      self.yolo_frame_roi_last_roi_depth=roi_depth

      #if (abs(roi_move)) > 500:
      #    t=Twist() 
      #    if roi_move > 0:
      #       t.angular.z=0.1  
      #    if roi_move < 0:
      #       t.angular.z=-0.1  
      
      #publish Twist to drive the follower

   def run(self):
      rate=rospy.Rate(1)
      while (not rospy.is_shutdown()):

         if len(self.sda.detections) >0:
            for d in self.sda.detections:
                rospy.loginfo("results ID=%d",d.results[0].id)
                if d.results[0].id == 0:
                   rospy.loginfo("Detected Human!")
                   x_center=d.bbox.center.x
                   y_center=d.bbox.center.y
                   xsize=d.bbox.size_x 
                   ysize=d.bbox.size_y
              
                   x1=int(x_center-(xsize/2))
                   y1=int(y_center-(ysize/2))
                   x2=int(x_center+(xsize/2))
                   y2=int(y_center+(ysize/2))
                  
                   roi=self.yolo_depth[y1:(y2+1),x1:(x2+1)]
                   rospy.loginfo(np.shape(roi))
                   try:
                      min_depth=np.amin(roi)
                      max_depth=np.amax(roi)
                      roi_depth=np.take(roi,roi.size // 2)
                      rospy.loginfo("min_depth=%s max_depth=%s center_depth=%s x1=%d y1=%d x2=%d y2=%d",min_depth,max_depth,roi_depth,x1,y1,x2,y2)
                      roi_x,roi_y=np.shape(roi)
                      self.follow_roi(roi_x,roi_y,roi_depth)

                   except ValueError:
                      pass

if __name__ == '__main__':

   follower=yoloFollower()
   follower.run()
