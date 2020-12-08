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
      self.level_distance=40.00
      self.nominal_diff=10.00
      self.drop_diff=50.00

      self.at_cliff=False
      
      rospy.init_node('cliff_watchdog')
      rospy.Subscriber(self.cliff_topic,Range,self.cliff_read_cb)
      self.cliff_pub=rospy.Publisher('cliff_state',Bool,latch=True,queue_size=2)

   def cliff_read_cb(self,data):

      distance = data.range
      if abs(self.distance-self.last_distance) > self.drop_diff:
         self.at_cliff=True
      self.last_distance = self.distance

   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
               
         self.cliff_pub.publish(self.at_cliff)

if __name__ == '__main__':

   cliff_node = Cliff("/sonar_front_center")
   cliff_node.run()

