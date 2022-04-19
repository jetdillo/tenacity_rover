#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist, Quaternion, Vector3, PoseStamped
from sensor_msgs.msg import Imu,Range

class Cliff():

   def __init__(self,sensor):

      self.cliff_topic = sensor

      self.distance = 0.0
      self.last_distance=0.0
      self.level_distance=340.00
      self.nominal_diff=20.00
      self.drop_diff=250.00
      self.hard_cliff=600.00
      self.dist_hist=[]
      self.hist_length=10
      self.hist_index=0
      self.at_cliff=False

      self.halt_msg=Twist()
      self.halt_msg.linear.x=0
      self.halt_msg.linear.y = 0
      self.halt_msg.linear.z = 0
      self.halt_msg.angular.x = 0
      self.halt_msg.angular.y = 0
      
      rospy.init_node('cliff_watchdog')
      rospy.Subscriber(self.cliff_topic,Range,self.cliff_read_cb)
      self.cliff_pub=rospy.Publisher('/cliff_state',Bool,latch=True,queue_size=2)
      self.halt_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10) 
 
   def cliff_read_cb(self,data):

      self.distance = data.range*1000

   def cliff_check(self):
      
      if self.hist_index >self.hist_length: 
         self.hist_index=0
      else:
         self.hist_index +=1

      self.dist_hist=[self.hist_index]=self.distance-self.last_distance
      self.nominal_diff = mean(dist_hist)
      self.last_distance = self.distance
      
      if abs(self.dist_hist[self.hist_index]) > self.drop_diff or if self.distance > hard_cliff:
         self.at_cliff=True
      else:
         self.at_cliff=False

   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         if self.cliff_check():
            self.halt_pub(self.halt_msg)       
         self.cliff_pub.publish(self.at_cliff)

if __name__ == '__main__':

   cliff_node = Cliff("/vlx/cliff")
   cliff_node.run()
