#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamixel_msgs.msg import JointState

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 

steer_min=205
steer_max=815

steer_state={}

class Corner:

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.steer_state={}
      self.twist = Twist()
      rospy.init_node("corner_steering_ctl")
      rospy.Subscriber("/cmd_vel",Twist,self.cmd_vel_cb)

      #We have 4 named steering controllers, create subscribers and publishers for them 

      self.steer_controllers=['front_left','front_right','rear_left','rear_right']
      for s in self.steer_controllers:
         steer_topic_str=s+"_controller/state"
         print steer_topic_str
         rospy.Subscriber(steer_topic_str,JointState,self.steering_state_cb)

      self.front_left_sp=rospy.Publisher('/front_left_controller/command',Float64,queue_size=1)
      self.front_right_sp=rospy.Publisher('/front_right_controller/command',Float64,queue_size=1)
      self.rear_left_sp=rospy.Publisher('/rear_left_controller/command',Float64,queue_size=1)
      self.rear_right_sp=rospy.Publisher('/rear_right_controller/command',Float64,queue_size=1)

   def cmd_vel_cb(self,data):
      self.twist.angular.z = data.angular.z
      
      print self.twist.angular.z
   
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
      self.steer_state[js.motor_ids[0]]=js

   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 
         if (self.twist.angular.z !=0): 
            front_newpos = (self.twist.angular.z * 1.5757)*-1
            rear_newpos = front_newpos *-1 
         else:
            front_newpos = 0
        
         print("front_newpos=%f rear_newpos=%f") % (front_newpos,rear_newpos) 

         self.front_left_sp.publish(front_newpos)
         self.front_right_sp.publish(front_newpos)
         self.rear_left_sp.publish(rear_newpos)
         self.rear_right_sp.publish(rear_newpos)
         time.sleep(1)

#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   cs_node = Corner()
   cs_node.run()

