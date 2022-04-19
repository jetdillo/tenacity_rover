#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist, Quaternion, Vector3, PoseStamped
from sensor_msgs.msg import Imu,Range

class ObstacleWatch():

   def __init__(self,sensor,name,nominal_diff,warn_dist,hard_dist):

      self.obstacle_topic = sensor
      self.obstacle_ctl = 'obstacle_command/'+name
      self.obstacle_name = name
      self.pub_name = 'obstacle_state/'+name
      self.distance = 0.0
      self.last_distance=0.0
      self.cur_distance=0.00
      self.nominal_diff=nominal_diff
      self.warn_dist=warn_dist
      self.hard_dist=hard_dist
      self.dist_hist=[]
      self.hist_length=10
      self.hist_index=0
      self.at_obstacle=False
      self.obstacle_state=0

      self.halt_msg=Twist()
      self.halt_msg.linear.x=0
      self.halt_msg.linear.y = 0
      self.halt_msg.linear.z = 0
      self.halt_msg.angular.x = 0
      self.halt_msg.angular.y = 0
      
      rospy.init_node('obstacle_watchdog')
      rospy.Subscriber(self.obstacle_topic,Range,self.obstacle_read_cb)
      rospy.Subscriber(self.obstacle_ctl,Range,self.obstacle_ctl_cb)
      self.obstacle_pub=rospy.Publisher(pub_name,Bool,latch=True,queue_size=2)
      self.halt_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10) 
 
   def obstacle_read_cb(self,data):
      
      self.distance = data.range

   def obstacle_ctl_cb(self,data):

      if data.data = 0:
         self.at_obstacle=False
         continue
      if data.data = 1:
         if self.last_distance > hard_obstacle:
            self.at_obstacle=False
         else:
            self.at_obstacle=True

   def obstacle_check(self):
      
      if self.hist_index >self.hist_length: 
         self.hist_index=0
      else:
         self.hist_index +=1

      self.dist_hist[self.hist_index]=abs(self.distance-self.last_distance)
      self.nominal_diff = mean(dist_hist)
      self.last_distance = self.distance
      
      if self.distance < hard_obstacle:
         self.at_obstacle=True
      else:
         self.at_obstacle=False

      self.obstacle_pub.publish(self.at_obstacle)
    
   def obstacle_clear(self):
      

   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         self.obstacle_check()
         if self.at_obstacle:_
            self.halt_pub(self.halt_msg)       
         self.obstacle_pub.publish(self.at_obstacle)

if __name__ == '__main__':
   sonars=['/ultrasound/fwd/left','/ultrasound/fwd/right']
   warn_dist=60
   hard_dist=20 
   obstacle_node = ObstacleWatch(sonars,warn_dist,hard_dist)
   obstacle_node.run()
