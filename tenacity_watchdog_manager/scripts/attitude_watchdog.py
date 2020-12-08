#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion, Vector3, PoseStamped
from sensor_msgs.msg import Imu

class Tilt():

   def __init__(self,imu_topic="/imu"):
      #init Dict to hold steering state for each servo
      self.imu = Imu()
      self.pitch_x = 0.00
      self.pitch_y = 0.00
      self.q = Quaternion()
      self.imu_topic=imu_topic
      self.warn_x = 0.17
      self.warn_y = 0.17
      self.max_x = 0.35
      self.max_y = 0.35

      rospy.init_node("attitude_watchdog")
      rospy.Subscriber(self.imu_topic,Imu,self.imu_read_cb)

   def imu_read_cb(self,data): 

       self.q.x = data.orientation.x 
       self.q.y = data.orientation.y
       self.q.z = data.orientation.z 
       self.q.w = data.orientation.w

       self.pitch_x = abs(self.q.x)
       self.pitch_y = abs(self.q.y)
       
   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         if ((self.pitch_x > 0.10 ) and (self.pitch_x < 0.17)):
            rospy.loginfo("Pitch Warning: X = %s",self.pitch_x)
         if ((self.pitch_x > 0.17 ) and (self.pitch_x < 0.35)):
            rospy.loginfo("BANK ANGLE X = %s",self.pitch_x)
         if (self.pitch_x >= 0.35): 
            rospy.loginfo("BANK CRITICAL X = %s",self.pitch_x)

         if ((self.pitch_y > 0.10 ) and (self.pitch_y < 0.17)):
            rospy.loginfo("Pitch Warning: Y = %s",self.pitch_y)
         if ((self.pitch_y > 0.17 ) and (self.pitch_y < 0.35)):
            rospy.loginfo("BANK ANGLE Y = %s",self.pitch_y)
         if (self.pitch_y >= 0.35): 
            rospy.loginfo("BANK CRITICAL Y = %s",self.pitch_y)

if __name__ == '__main__':

   i_node = Tilt("gx5/imu/data")
   i_node.run()

