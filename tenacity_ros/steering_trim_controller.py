#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
from dynamixel_msgs.msg import JointState

#Generic steering interface for non-teleop nodes to control the Dynamixels that control the corner steering.
#Reads off /cmd_vel like a normal control interface and publishes those values to 

steer_min=205
steer_max=815

steer_state={}

steer_mode=0

class Wander:

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.steer_state={}
      self.twist = Twist()
      self.sonar_range={'front_left':0.0,'front_right':0.0}
      self.cliff_range = Range() 
      self.sonar_wander=True
      self.cliff_flag=False
      self.throttle=0
      self.max_speed=0.5

      #self.outer_range=rospy.get_param("/fwd_ranger_params/outer_range")
      #self.warn_range=rospy.get_param("/fwd_ranger_params/warn_range")
      #self.critical_range=rospy.get_param("/fwd_ranger_params/critical_range")
      
      self.outer_range=1.00 
      self.warn_range=0.4
      self.critical_range=0.15

      rospy.init_node("sonar_trim")

      #self.sonar=['/ultrasound/front_front_left','/ultrasound/front_front_right']
      self.sonar=['/ultrasound/front_left','/ultrasound/front_right']
      self.cliff='/vlx/front_cliff'

      rospy.Subscriber(self.sonar[0],Range,self.front_left_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.front_right_sonar_cb)
      rospy.Subscriber(self.cliff,Range,self.cliff_cb)

      rospy.Subscriber('/ackermann_drive_controller/cmd_vel',Twist,self.cmd_vel_cb)
         
      self.drive=rospy.Publisher('/ackermann_drive_controller/cmd_vel',Twist,queue_size=10)

   def cmd_vel_cb(self,data):
      self.twist.linear.x = data.linear.x
      self.twist.angular.z = data.angular.z
      
   def front_left_sonar_cb(self,data):
      r=Range()
      r=data
      self.sonar_range['front_left']=r.range
   
   def front_right_sonar_cb(self,data):
      r=Range()
      r=data
      self.sonar_range['front_right']=r.range

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
               #We assume that an outside controller/planner is controlling linear velocity. 

               sonar_diff = abs(self.sonar_range['front_left'] - self.sonar_range['front_right'])
               print("cliff_flag=%s sonar_diff=%s front_left=%s,front_right=%s" % (self.cliff_flag,sonar_diff,self.sonar_range['front_left'],self.sonar_range['front_right']))
               if sonar_diff > 0.25:
                  if self.sonar_range['front_left'] > self.sonar_range['front_right']:
                     rospy.loginfo("LEFT TURN")
                     t.angular.z=0.1
                  if self.sonar_range['front_right'] > self.sonar_range['front_left']:
                     t.angular.z =-0.1  
                     rospy.loginfo("RIGHT TURN")
               else:
                  if self.sonar_range['front_left'] < self.critical_range:
                     t.angular.z =0.3
                     rospy.loginfo("OBSTACLE LEFT - RIGHT TURN")
                  
                  elif self.sonar_range['front_right'] < self.critical_range:
                     t.angular.z =-0.3
                     rospy.loginfo("OBSTACLE RIGHT - LEFT TURN")
                  else:
                     t.angular.z = self.twist.angular.z

               #Center sonar not on the real robot  
               #if self.sonar_range['center'] < self.warn_range:
               #    if self.sonar_range['front_left'] > self.sonar_range['center']:
               #        t.angular.z=0.1
               #    if self.sonar_range['front_right'] > self.sonar_range['center']:
               #        t.angular.z=-0.1

               t.linear.x=self.twist.linear.x
               self.drive.publish(t)  
            #Otherwise a cliff_alarm has been triggered. 
            else:
               t=Twist()
               self.drive.publish(t)           
                      
if __name__ == '__main__':

   w = Wander()
   w.run()
