#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64,Bool
from geometry_msgs.msg import Twist,TwistStamped
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
      self.curTwist = Twist()
      self.sonar_range={'forward_left':0.0,'forward_right':0.0,'forward_center':0.0}
      self.sonar_score={'forward_left':0,'forward_right':0,'forward_center':0}
      self.sonar_last={'forward_left':0,'forward_right':0,'forward_center':0}
      self.cliff_range = Range() 
      self.sonar_wander=True
      self.cliff_flag=False
      self.throttle=0
      self.max_speed=0.5
      self.min_speed=0.1

      #self.outer_range=rospy.get_param("/fwd_ranger_params/outer_range")
      #self.warn_range=rospy.get_param("/fwd_ranger_params/warn_range")
      #self.critical_range=rospy.get_param("/fwd_ranger_params/critical_range")
      
      self.outer_range=1.40 
      self.warn_range=0.60 
      self.critical_range=0.30

      rospy.init_node("sonar_wander")

      self.sonar=['/sensors/ultrasound/left','/sensors/ultrasound/right','/sensors/ultrasound/forward']
      self.cliff='/sensors/ultrasound/cliff'

      rospy.Subscriber(self.sonar[0],Range,self.left_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.right_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.forward_sonar_cb)
      rospy.Subscriber(self.cliff,Range,self.cliff_cb)

      self.drive=rospy.Subscriber('/ackermann_drive_controller/cmd_vel_out',TwistStamped,self.cmd_vel_cb)
         
      self.drive=rospy.Publisher('/ackermann_drive_controller/cmd_vel',Twist,queue_size=10)

   def cmd_vel_cb(self,data):
      self.curTwist.angular.z = data.twist.angular.z
      self.curTwist.linear.x = data.twist.linear.x
      self.curTwist.linear.y = data.twist.linear.y
      
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
      t.linear.x=0
      t.linear.z=0
      t.angular.z =0.0
      t.angular.x=0.0
      
      avoidance_trim=0
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 
         
         if self.sonar_wander:
            if not self.cliff_flag:
               #sonar sensors aren't really meant for navigation
               #just obstacle avoidance
               #ground_speed=self.set_speed()
               avoidance_trim=self.score_sonars()
               t=self.curTwist
               if abs(t.angular.z) <= 0.3 and avoidance_trim >0:
                  t.angular.z+=avoidance_trim
               else:
                  t.angular.z=0
               t.linear.x=ground_speed
               self.drive.publish(t) 

   def score_sonars(self):

      trim_dir=0
#Weight heavily towards center 
#Unless one of the sides is dangerously close, we want to keep going forward
#Probably a better way to do this but I need to look up costs for loops and dict look ups vs. straight waterfall
      
      for k in self.sonar_score.keys():
         self.sonar_score[k]=0

      for k in self.sonar_range.keys():
         if self.sonar_range[k] <  self.warn_range:
            self.sonar_score[k] -=1
            if self.sonar_range[k] < self.critical_range:
               self.sonar_score[k] -=5
         
#Tie-breakers: apply rewards/penalties for one side being more favorable than the other. 
#S      
      if self.sonar_range['forward_right'] > self.sonar_range['forward_left']:
         self.sonar_score['forward_right'] += 1
         self.sonar_score['forward_left'] -= 1

      if self.sonar_range['forward_right'] > self.sonar_range['forward_center']:
         self.sonar_score['forward_right'] += 1
         self.sonar_score['forward_center'] -= 1

      if self.sonar_range['forward_left'] > self.sonar_range['forward_right']:
         self.sonar_score['forward_left'] += 1
         self.sonar_score['forward_right'] -= 1

      if self.sonar_range['forward_left'] > self.sonar_range['forward_center']:
         self.sonar_score['forward_left'] += 1
         self.sonar_score['forward_center'] -= 1
  
#Add it up,add it up 
         
      if self.sonar_score['forward_center'] > self.sonar_score['forward_left'] and \
         self.sonar_score['forward_center'] > self.sonar_score['forward_right']:
         trim_dir=0

      if self.sonar_score['forward_left'] < self.sonar_score['forward_center'] and \
         self.sonar_score['forward_left'] < self.sonar_score['forward_right']:
         trim_dir=-0.1

      if self.sonar_score['forward_right'] < self.sonar_score['forward_center'] and \
         self.sonar_score['forward_right'] < self.sonar_score['forward_left']:
         trim_dir=0.1
     
      rospy.loginfo("left score: %s center score: %s right score: %s",self.sonar_score['forward_left'],self.sonar_score['forward_center'],self.sonar_score['forward_right'] )
      
      #current_values are now last ones
      self.sonar_last=self.sonar_range
       
      return trim_dir

   def set_speed(self):
               throttle=self.curTwist.linear.x

               #rospy.loginfo("forward_range:%s warn_range:%s current_speed:%s" ,self.sonar_range['forward_center'],self.warn_range,throttle)

               if self.sonar_range['forward_center'] > self.warn_range: 

                     if throttle < self.max_speed:
                        throttle+=0.1
                     else:
                        throttle=self.max_speed
               else: 
                  if throttle < (self.max_speed/2)*-1:
                     throttle=(self.max_speed/2)*-1
                  else:
                     throttle-=0.1
              
               return throttle

#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   w = Wander()
   w.run()
