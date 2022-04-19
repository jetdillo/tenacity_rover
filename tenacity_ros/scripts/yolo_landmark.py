#!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2
import numpy as np
import sys

import simplejson

from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Point,PoseWithCovarianceStamped,Vector3
from sensor_msgs.msg import Image,NavSatFix,Imu, MagneticField

from vision_msgs.msg import ObjectHypothesis,BoundingBox2D
from depthai_ros_msgs.msg import SpatialDetection
from depthai_ros_msgs.msg import SpatialDetectionArray

class yoloGeoAnnotate:

   def __init__(self):
   
      rospy.init_node("yoloGeoAnnotate")

      self.yolo_str="/yolov4_publisher/color/yolov4_Spatial_detections"
      self.yolo_obj_topic="/yolov4/detected_objects"

      self.yolo_labels=[]
      self.yolo_oi=['person','stop sign','backpack','fire hydrant','apple','orange','laptop','cell phone']

      self.yolo_rgb="/yolov4_publisher/color/image"
      self.yolo_rgb_out="/yolov4_publisher/color/annotated_image"
      self.yolo_img=Image()
      self.yolo_img_dim_x=0
      self.yolo_img_dim_y=0

      self.yolo_nav_device="/gx5"
      self.yolo_gps_topic=self.yolo_nav_device+"/gps/fix"
      self.yolo_imu_topic=self.yolo_nav_device+"/imu/data"

      self.sda=SpatialDetectionArray()
      self.detected_object=SpatialDetection()
      self.cvb=CvBridge()

      self.imu = Imu()
      self.snf = SatNavFix()

      self.pitch_x = 0.00
      self.pitch_y = 0.00

      self.q = Quaternion()

      rospy.Subscriber(self.yolo_str,SpatialDetectionArray, self.sda_proc_cb)
      rospy.Subscriber(self.yolo_rgb,Image, self.yolo_img_cb)
      rospy.Subscriber(self.yolo_gps_topic,NavSatFix, self.gps_cb)
      rospy.Subscriber(self.yolo_imu_topic,Imu, self.imu_cb)
      self.boxed_image=rospy.Publisher(self.yolo_rgb_out,Image,queue_size=1)

      with open(label_file) as lf:
          yj = json.load(lf)

      self.obj_labels= yj.['mappings']['labels']
       
   def sda_proc_cb(self,data):
      self.sda=data

   def yolo_img_cb(self,data):
      self.yolo_img=data 
      self.yolo_img_dim_y=self.yolo_img.height
      self.yolo_img_dim_x=self.yolo_img.width
      print("img_x=%d img_y=%d" % (self.yolo_img.height,self.yolo_img.width))

   def yolo_pindrop(self,detected_object):

   #Take object data and current GPS fix coords and put that in a record that could be recalled/re-looked up later
   #It would be cool to put this in a KMZ, but something simpler should be fine for now
   #JSON consisting of: {{lat,long},{Z dist from curpos},{object_id},{confidence}}

        

   def run(self):
      rate=rospy.Rate(1)
      while (not rospy.is_shutdown()):

         if len(self.sda.detections) >0:
            for d in self.sda.detections:
               rospy.loginfo("DetectedObject: %s with confidence %s at X:%s Y:%s Center_x:%s Center_y:%s",d.results[0].id,d.results[0].score,d.bbox.size_x,d.bbox.size_y,d.bbox.center.x,d.bbox.center.y)
               cv_image = self.cvb.imgmsg_to_cv2(self.yolo_img,"bgr8")

               x_center=int(d.bbox.center.x)
               y_center=int(d.bbox.center.y)
               xsize=d.bbox.size_x 
               ysize=d.bbox.size_y
              
               x1=int(x_center-(xsize/2))
               y1=int(y_center-(ysize/2))
               x2=int(x_center+(xsize/2))
               y2=int(y_center+(ysize/2))
               rospy.loginfo("x1=%d y1=%d x2=%d y2=%d",x1,y1,x2,y2)
               cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,255,0),2)
               cv2.rectangle(cv_image,(0,0),(self.yolo_img_dim_x,self.yolo_img_dim_y),(255,0,0),2)
               cv2.rectangle(cv_image,((x_center-2),(y_center-2)),((x_center+2),(y_center+2)),(0,0,255),2)
               #cv2.rectangle(cv_image,(384,0),(510,128),(0,255,0),3)
               self.boxed_image.publish(self.cvb.cv2_to_imgmsg(cv_image,"bgr8"))

               if d.results[0].id in self.yolo_objects and d.results[0].score > 0.5:
                   self.yolo_pin_drop(d)

if __name__ == '__main__':

   overlay=yoloImageOverlay()
   overlay.run()
