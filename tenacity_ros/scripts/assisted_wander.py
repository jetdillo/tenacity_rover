#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
from dynamixel_msgs.msg import JointState

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 

steer_min=205
steer_max=815

steer_state={}

steer_mode=0

class Wander:

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.steer_state={}
      self.twist = Twist()
      self.sonar_range={'forward_left':0.0,'forward_right':0.0}
      self.cliff_range = Range() 
      self.sonar_wander=True
      self.cliff_flag=False
      self.throttle=0
      self.max_speed=0.5

      #self.outer_range=rospy.get_param("/fwd_ranger_params/outer_range")
      #self.warn_range=rospy.get_param("/fwd_ranger_params/warn_range")
      #self.critical_range=rospy.get_param("/fwd_ranger_params/critical_range")
      
      self.outer_range=140 
      self.warn_range=60 
      self.critical_range=30

      rospy.init_node("sonar_wander")

      self.sonar=['/ultrasound/front_left','/ultrasound/front_right']
      self.cliff='/vlx/front_cliff'

      rospy.Subscriber(self.sonar[0],Range,self.left_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.right_sonar_cb)
      rospy.Subscriber(self.cliff,Range,self.cliff_cb)
         
      self.drive=rospy.Publisher('/ackermann_drive_controller/cmd_vel',Twist,queue_size=10)

   def cmd_vel_cb(self,data):
      self.twist.angular.z = data.angular.z
      
   def left_sonar_cb(self,data):
      r=Range()
      r=data
      self.sonar_range['forward_left']=r.range
   
   def right_sonar_cb(self,data):
      r=Range()
      r=data
      self.sonar_range['forward_right']=r.range

   def cliff_cb(self,data):
      self.cliff_range = data.data
      if self.cliff_range > 0.85:
         self.cliff_flag=True

   def run(self): 
      t=Twist() 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 
         
         if self.sonar_wander:
            if not self.cliff_flag:
               #sonar sensors aren't really meant for navigation
               #just obstacle avoidance
               sonar_diff = abs(self.sonar_range['forward_left'] - self.sonar_range['forward_right'])
               print("cliff_flag=%s sonar_diff=%s forward_left=%s,forward_right=%s" % (self.cliff_flag,sonar_diff,self.sonar_range['forward_left'],self.sonar_range['forward_right']))
               if sonar_diff < 0.25:
                  if self.sonar_range['forward_left'] > self.warn_range: 
                     if self.throttle < self.max_speed:
                        self.throttle+=0.1
                     else:
                        self.throttle=self.max_speed
                  else: 
                     if self.sonar_range['forward_left'] < self.critical_range:
                        self.throttle=-0.1
                     else:
                        self.throttle=0.1

                  t.linear.x=self.throttle
                  t.linear.y=0
                  t.linear.z=0
                  t.angular.z =0.0
                  t.angular.x=0.0
                  t.angular.y=0.0
                  self.twist=t
                  self.drive.publish(self.twist)           
               else:
                  if self.sonar_range['forward_left'] > self.sonar_range['forward_right']:
                     rospy.loginfo("LEFT TURN")
                  if self.sonar_range['forward_right'] > self.sonar_range['forward_left']:
                     t.angular.z -=0.1  
                     rospy.loginfo("RIGHT TURN")
               
                  if self.sonar_range['forward_left'] < self.critical_range:
                     t.angular.z +=0.3
                     rospy.loginfo("OBSTACLE LEFT - RIGHT TURN")
               
                  if self.sonar_range['forward_right'] < self.critical_range:
                     t.angular.z -=0.3
                     rospy.loginfo("OBSTACLE RIGHT - LEFT TURN")
                  
                  if self.throttle > 0:
                     self.throttle -=0.1
                  else:
                     self.throttle=0.1

                  t.linear.x=self.throttle
                
                  #turn angles are zero and fwd velocity gets a bump or is stable
                  #It occurs to me that there's no actual service for doing a commanded corner_twist turn
                  #We should write a service to do that or investigate how the actual ackermann_controller works

               self.twist=t
               self.drive.publish(self.twist)  
            #Otherwise a cliff_alarm has been triggered. 
            else:
               self.twist=Twist()
               self.drive.publish(self.twist)           
                      

#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   w = Wander()
   w.run()
