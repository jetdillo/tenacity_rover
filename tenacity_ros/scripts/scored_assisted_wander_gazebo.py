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
      self.sonar_range={'forward_left':0.0,'forward_right':0.0,'forward_center':0.0}
      self.cliff_range = Range() 
      self.sonar_wander=True
      self.cliff_flag=False
      self.throttle=0
      self.max_speed=0.5
      self.min_speed=0.1

      #self.outer_range=rospy.get_param("/fwd_ranger_params/outer_range")
      #self.warn_range=rospy.get_param("/fwd_ranger_params/warn_range")
      #self.critical_range=rospy.get_param("/fwd_ranger_params/critical_range")
      
      self.outer_range=140 
      self.warn_range=60 
      self.critical_range=30

      rospy.init_node("sonar_wander")

      self.sonar=['/sensors/ultrasound/left','/sensors/ultrasound/right','/sensors/ultrasound/forward']
      self.cliff='/sensors/ultrasound/cliff'

      rospy.Subscriber(self.sonar[0],Range,self.left_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.right_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.forward_sonar_cb)
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
   
   def forward_sonar_cb(self,data):
      r=Range()
      r=data
      self.sonar_range['forward_center']=r.range


   def cliff_cb(self,data):
      r=Range()
      r=data
      self.cliff_range = r.range
      if self.cliff_range > 100:
         self.cliff_flag=True

   def run(self): 
      t=Twist() 
      t.linear.y=0
      t.linear.z=0
      t.angular.z =0.0
      t.angular.x=0.0
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 
         
         if self.sonar_wander:
            if not self.cliff_flag:
               #sonar sensors aren't really meant for navigation
               #just obstacle avoidance

               if self.sonar_range['forward_center'] > self.warn_range: 

                     if self.throttle < self.max_speed:
                        self.throttle+=0.1
                     else:
                        self.throttle=self.max_speed
               else:
                  if self.sonar_range['forward_center'] > self.critical_range:
                     self.throttle =self.max_speed/2 
                  else:
                     self.throttle=self.min_speed
 
               t.linear.x=self.throttle
               self.twist=t
               self.drive.publish(self.twist)  
            #Otherwise a cliff_alarm has been triggered. 
            else:
               t.linear.x=0
               t.linear.y=0
               t.linear.z=0
               t.angular.x = 0
               t.angular.y = 0    
               t.angular.z = 0

               self.twist=t
               self.drive.publish(self.twist)           
                      

#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   w = Wander()
   w.run()
