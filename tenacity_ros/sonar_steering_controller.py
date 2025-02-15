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
steer_topic=''
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

      self.outer_range=1.00 
      self.warn_range=0.4
      self.critical_range=0.15

      rospy.init_node("sonar_trim")

      self.sonar=['/ultrasound/forward_left','/ultrasound/forward_right']
      self.cliff='/vlx/front_cliff'

      rospy.Subscriber(self.sonar[0],Range,self.front_left_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.front_right_sonar_cb)
      rospy.Subscriber(self.cliff,Range,self.cliff_cb)

      if not 'ackerman' in rospy.get_published_topics():
         steer_topic='/cmd_vel'
      else:
         steer_topic='/ackermann_drive_controller/cmd_vel'

      rospy.Subscriber(steer_topic,Twist,self.cmd_vel_cb)
         
      self.drive=rospy.Publisher(steer_topic,Twist,queue_size=10)

   def cmd_vel_cb(self,data):
      self.twist.linear.x = data.linear.x
      self.twist.angular.z = data.angular.z
      
   def front_left_sonar_cb(self,data):
      r=Range()
      r=data
      self.sonar_range['forward_left']=r.range
   
   def front_right_sonar_cb(self,data):
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
               #We assume that an outside controller/planner is controlling linear velocity. 

               sonar_diff = abs(self.sonar_range['forward_left'] - self.sonar_range['forward_right'])
               print("cliff_flag=%s sonar_diff=%s forward_left=%s,forward_right=%s" % (self.cliff_flag,sonar_diff,self.sonar_range['forward_left'],self.sonar_range['forward_right']))
               if self.sonar_range['forward_left'] > 0.01 and self.sonar_range['forward_right'] > 0.01:

                  if self.sonar_range['forward_left'] < self.critical_range and
                     self.sonar_range['forward_right'] < self.critical_range:
                        t.angular.z = 0.0
                        t.linear.x = -0.25
                        while(self.sonar_range['forward_left'] < self.critical_range and
                                self.sonar_range['forward_right'] < self.critical_range) :
                              self.drive.publish(t)  
                              rospy.loginfo("BACK UP")

                  if self.sonar_range['forward_left'] < self.critical_range:
                        t.angular.z =-0.3
                        t.linear.x=self.twist.linear.x
                        self.drive.publish(t)  
                        rospy.loginfo("OBSTACLE LEFT - RIGHT TURN")
                  
                  if self.sonar_range['forward_right'] < self.critical_range:
                        t.angular.z =0.3
                        t.linear.x=self.twist.linear.x
                        self.drive.publish(t)  
                        rospy.loginfo("OBSTACLE RIGHT - LEFT TURN")
                  #else:
                  #      t.angular.z = self.twist.angular.z
                  #      t.linear.x=self.twist.linear.x

               #t.linear.x=self.twist.linear.x
               #self.drive.publish(t)  
            #Otherwise a cliff_alarm has been triggered. 
            else:
               t=Twist()
               self.drive.publish(t)           
               rospy.sleep(1)       
if __name__ == '__main__':

   w = Wander()
   w.run()
