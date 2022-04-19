#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
from dynamixel_msgs.msg import JointState

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 

steer_min=205
steer_max=815

steer_state={}

steer_mode=0
steer_debounce=25

class Wander:

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.steer_state={}
      self.twist = Twist()
      self.sonar_ranges=[0.0,0.0,0.0]
      self.sonar_steer=True
      self.cliff=False

      rospy.init_node("corner_dodge_ctl")
      rospy.Subscriber("/cmd_vel",Twist,self.cmd_vel_cb)
      rospy.Subscriber("/joy", Joy, self.joy_cb)
      #We have 4 named steering controllers, create subscribers and publishers for them 

      self.steer_controllers=['front_left','front_right','rear_left','rear_right']
      self.sonar=['/ultrasound/front_left','/ultrasound/front_right','/ultrasound/front_center']

      for s in self.steer_controllers:
         steer_topic_str=s+"_controller/state"
         print steer_topic_str
         rospy.Subscriber(steer_topic_str,JointState,self.steering_state_cb)
         self.steer_state[s]=JointState()
 
      rospy.Subscriber(self.sonar[0],Range,self.left_sonar_cb)
      rospy.Subscriber(self.sonar[1],Range,self.right_sonar_cb)
      rospy.Subscriber(self.sonar[2],Range,self.center_sonar_cb)
         
      rospy.Subscriber("/cliff_state",Bool,self.cliff_cb)
      
      #self.drive=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
      self.front_left_sp=rospy.Publisher('/front_left_controller/command',Float64,queue_size=1)
      self.front_right_sp=rospy.Publisher('/front_right_controller/command',Float64,queue_size=1)
      self.rear_left_sp=rospy.Publisher('/rear_left_controller/command',Float64,queue_size=1)
      self.rear_right_sp=rospy.Publisher('/rear_right_controller/command',Float64,queue_size=1)

   def cmd_vel_cb(self,data):
      self.twist.angular.z = data.angular.z
      self.twist.linear.x = data.linear.x

#This looks a bit redundant but I think is better because we only have the 3 sonars
#and it's more direct, more readable and saves a bunch of lookups 
      
   def left_sonar_cb(self,data):
      self.sonar_ranges[0]=data.range
      print("left: %f" % data.range)

   def right_sonar_cb(self,data):

      self.sonar_ranges[1]=data.range
      print("right: %f" % data.range)

   def center_sonar_cb(self,data):
      self.sonar_ranges[2]=data.range
      print("center: %f" % data.range)

   def cliff_cb(self,data):
   
      self.cliff = data.data

   def joy_cb(self,data):
   # Use this for processing buttons that do a certain "autonomy by macro"
 
     j = Joy()
     j = data
 
   def twist_to_steer_angle(self):
      
      newpos=0
      
      if (self.twist.angular.z !=0): 
         newpos = self.twist.angular.z * 1.5757
      else:
         newpos = 0
       
      return newpos
 
   #Publish steer_angle to the front servos
   #Publish the mirror of steer_angle to the rear ones

   def steering_state_cb(self,data):
      js=JointState()
      js.name=data.name
      js.motor_ids=data.motor_ids
      js.current_pos=data.current_pos
      self.steer_state[js.name]=js

   def run(self): 
      rate= rospy.Rate(10)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 
         front_curpos = self.steer_state['front_right'].current_pos
         if self.sonar_steer:
            if not self.cliff:
               #sonar sensors aren't really meant for navigation
               #just obstacle avoidance

               rospy.loginfo("left_sonar:%f right_sonar:%f center_sonar:%f",self.sonar_ranges[0],self.sonar_ranges[1],self.sonar_ranges[2])
               if self.sonar_ranges[0] < 0.3 and self.sonar_ranges[0] > 0:
                  rospy.loginfo("HARD RIGHT TURN:left_sonar:%s right_sonar:%s",self.sonar_ranges[0],self.sonar_ranges[1])
                  front_newpos = 1.05
                  rear_newpos = front_newpos *-1
                  self.front_left_sp.publish(front_newpos)
                  self.front_right_sp.publish(front_newpos)
                  self.rear_left_sp.publish(rear_newpos)
                  self.rear_right_sp.publish(rear_newpos)

               if self.sonar_ranges[1] < 0.3 and self.sonar_ranges[1] > 0: 
                  rospy.loginfo("HARD LEFT TURN:left_sonar:%s right_sonar:%s",self.sonar_ranges[0],self.sonar_ranges[1])
                  front_newpos = -1.05
                  rear_newpos = front_newpos *-1
                  self.front_left_sp.publish(front_newpos)
                  self.front_right_sp.publish(front_newpos)
                  self.rear_left_sp.publish(rear_newpos)
                  self.rear_right_sp.publish(rear_newpos)

               if self.sonar_ranges[2] < 0.75 :
                  speed_factor=float(self.sonar_ranges[2])
                  rospy.loginfo("SLOWING from %s to %s",self.twist.linear.x,self.twist.linear.x*speed_factor)

#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   cs_node = Wander()
   time.sleep(2)
   cs_node.run()
