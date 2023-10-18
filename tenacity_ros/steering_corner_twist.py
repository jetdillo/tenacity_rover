#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist
from dynamixel_msgs.msg import JointState as JointState_DM
from sensor_msgs.msg import JointState as JointState_SM
from openmv_flow.msg import OpticalDrift 

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 
#Generic controller/interface for autonomous actions


steer_min=205
steer_max=815

steer_state={}

class Trim:

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.steer_mode=0
      self.auto_mode=1
      self.steer_state={}
      self.twist = Twist()
      self.jss = JointState_SM()
      self.od_msg = OpticalDrift()
      rospy.init_node("corner_steering_trim")
      rospy.Subscriber("/cmd_vel",Twist,self.cmd_vel_cb)
      rospy.Subscriber("/joy", Joy, self.joy_cb)

      rospy.Subscriber("/optical_drift",OpticalDrift,self.optical_drift_cb)

      #We have 4 named steering controllers, create subscribers and publishers for them 

      self.jss.name=self.steer_controllers=['front_left','front_right','rear_left','rear_right']

      self.jss.position=[0.0,0.0,0.0,0.0]
      self.jss.velocity=[0.0,0.0,0.0,0.0]

      for s in self.steer_controllers:
         self.steer_state[s]=0.0

      #print(self.jss.name)

      for s in self.steer_controllers:
         steer_state_str=s+"_controller/state"
         #print steer_state_str
         rospy.Subscriber(steer_state_str,JointState_DM,self.steering_state_cb)

      self.front_left_sp=rospy.Publisher('/front_left_controller/command',Float64,queue_size=1)
      self.front_right_sp=rospy.Publisher('/front_right_controller/command',Float64,queue_size=1)
      self.rear_left_sp=rospy.Publisher('/rear_left_controller/command',Float64,queue_size=1)
      self.rear_right_sp=rospy.Publisher('/rear_right_controller/command',Float64,queue_size=1)

      self.js_repub=rospy.Publisher('joint_states',JointState_SM,queue_size=1)

   def cmd_vel_cb(self,data):
      self.twist.angular.z = data.angular.z
      
      #print self.twist.angular.z

   def optical_drift_cb(self,data):
       self.od_msg = data

   def joy_cb(self,data):
       self.joy_msg = data

   def twist_to_steer_angle(self):
      
      newpos=0
      
      newpos = self.twist.angular.z * 1.5757
       
      return newpos
 
   #Publish steer_angle to the front servos
   #Publish the mirror of steer_angle to the rear ones

   def steering_state_cb(self,data):

      jsp=[]
      js=JointState_DM()
      js.name=data.name
      js.motor_ids=data.motor_ids
      js.current_pos=data.current_pos
      self.steer_state[js.name]=js.current_pos

      for jp in self.jss.name:
         #print(jp)
         jsp.append(self.steer_state[jp])
      
      for j in range(0,len(jsp)):
         self.jss.position[j]=jsp[j]

   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 

         #if self.steer_mode == 0 :
         if self.auto_mode == 1 :
            if (abs(self.twist.angular.z) >0.05): 
            #We actually mean to turn here...
               front_newpos = (self.twist.angular.z * 1.5757)*-1
               rear_newpos = front_newpos *-1 
            else:
               #We're drifting, calculate how much to turn to get us back on course
               #Figure out actual values but guess for now. 
               #Left drift is negative, right drift is positive. 
               ##Left Z is positive, right Z is negative

               rospy.loginfo("Steering Drift Detected: %s",self.od_msg)
            #print("front_newpos=%f rear_newpos=%f") % (front_newpos,rear_newpos) 

            self.front_left_sp.publish(front_newpos)
            self.front_right_sp.publish(front_newpos)
            self.rear_left_sp.publish(rear_newpos)
            self.rear_right_sp.publish(rear_newpos)
            self.js_repub.publish(self.jss)
            
            time.sleep(1)

if __name__ == '__main__':

   ts_node = Trim()
   ts_node.run()
