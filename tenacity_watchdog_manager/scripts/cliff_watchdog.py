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
      self.level_distance=0.6
      self.nominal_cliff=0.85
      self.nominal_diff=0.25
      self.drop_diff=0.5
      self.cliff_timeout=1.0
      self.alarm=False
      self.at_cliff=False
      
      rospy.init_node('cliff_watchdog')
      rospy.Subscriber(self.cliff_topic,Range,self.cliff_read_cb)
      self.cliff_pub=rospy.Publisher('cliff_state',Bool,queue_size=10)

   def cliff_read_cb(self,data):
      
      self.distance = data.range

   def run(self): 
      last_time=0
      cliff_time=0
      rate= rospy.Rate(5)
      while (not rospy.is_shutdown()):
         if self.distance > self.level_distance+self.nominal_diff :
            self.alarm=True
              #print("Cliff WARN: cur_range:%s"  % self.distance)
         else:
            self.alarm=False
            cliff_time = time.time()
            #print("Cliff range: cur_range:%s"  % self.distance)
    
         if self.alarm:
            if abs(cliff_time - time.time()) >self.cliff_timeout:
               self.at_cliff=True 
               self.cliff_pub.publish(self.at_cliff)
               #print("CLIFF ALARM! range:%s drop_limit:%s nominal_cliff:%s time: %s " % (self.distance,self.drop_diff,self.nominal_cliff,abs(cliff_time - time.time())))
            else:
                self.at_cliff=False
         self.cliff_pub.publish(self.at_cliff)

if __name__ == '__main__':

   cliff_node = Cliff("/vlx/front_cliff")
   cliff_node.run()
