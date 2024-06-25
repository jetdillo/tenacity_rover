#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamixel_msgs.msg import JointState as JointState_DM
from sensor_msgs.msg import JointState as JointState_SM

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 

steer_min=205
steer_max=815

steer_state={}

class Corner:

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.steer_mode=0
      self.steer_state={}
      self.twist = Twist()
      self.jss = JointState_SM()
      rospy.init_node("corner_steering_ctl")
      rospy.Subscriber("/cmd_vel",Twist,self.cmd_vel_cb)
      #rospy.Subscriber("/joy", Joy, self.joy_cb)
      #We have 4 named steering controllers, create subscribers and publishers for them 

      self.jss.name=self.steer_controllers=['front_left','front_right','rear_left','rear_right']

      self.jss.position=[0.0,0.0,0.0,0.0]
      self.jss.velocity=[0.0,0.0,0.0,0.0]

      for s in self.steer_controllers:
         self.steer_state[s]=0.0

      #print(self.jss.name)

      for s in self.steer_controllers:
         steer_topic_str=s+"_controller/state"
         #print steer_topic_str
         rospy.Subscriber(steer_topic_str,JointState_DM,self.steering_state_cb)

      self.front_left_sp=rospy.Publisher('/front_left_controller/command',Float64,queue_size=1)
      self.front_right_sp=rospy.Publisher('/front_right_controller/command',Float64,queue_size=1)
      self.rear_left_sp=rospy.Publisher('/rear_left_controller/command',Float64,queue_size=1)
      self.rear_right_sp=rospy.Publisher('/rear_right_controller/command',Float64,queue_size=1)

      self.js_repub=rospy.Publisher('joint_states',JointState_SM,queue_size=1)

   def cmd_vel_cb(self,data):
      #We just deal with data.angular here because the data.linear values
      #are consumed inside the motor controller code 
      self.twist.angular.z = data.angular.z
      
      #print self.twist.angular.z

   def joy_cb(self,data):
      
      if data.axes[6] == -1.0:
         self.steer_mode = 2
      elif data.axes[6] == 1.0:
         self.steer_mode = 1
      else:
         self.steer_mode = 0

      print(data.axes),
      print(self.steer_mode)
   
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

   def exercise(self,exer_count):
      angles=[0.64,-0.64]
      #dir=[1,-1]
      for p in range(0,exer_count):

          leftside=angles[0]
          rightside=angles[1]
         
          self.front_left_sp.publish(leftside)
          self.front_right_sp.publish(rightside)
          self.rear_left_sp.publish(leftside)
          self.rear_right_sp.publish(rightside)
          
          time.sleep(2)
          
          leftside=angles[1]
          rightside=angles[0]

          self.front_left_sp.publish(leftside)
          self.front_right_sp.publish(rightside)
          self.rear_left_sp.publish(leftside)
          self.rear_right_sp.publish(rightside)

   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
         front_newpos=0
         rear_newpos=0 

         if self.steer_mode == 0:
            if (self.twist.angular.z !=0): 
               front_newpos = (self.twist.angular.z * 1.5757)*-1
               rear_newpos = front_newpos *-1 
            else:
               front_newpos = 0
        
            #print("front_newpos=%f rear_newpos=%f") % (front_newpos,rear_newpos) 

            self.front_left_sp.publish(front_newpos)
            self.front_right_sp.publish(front_newpos)
            self.rear_left_sp.publish(rear_newpos)
            self.rear_right_sp.publish(rear_newpos)
            self.js_repub.publish(self.jss)
            
            time.sleep(1)
     
         if not self.steer_mode == 0:
         
            left_front_pos = 0.64
            right_front_pos = -0.64
            left_rear_pos = -0.64
            right_rear_pos = 0.64
            self.front_left_sp.publish(left_front_pos)
            self.front_right_sp.publish(right_front_pos)
            self.rear_left_sp.publish(left_rear_pos)
            self.rear_right_sp.publish(right_rear_pos)
            time.sleep(1)

if __name__ == '__main__':

   cs_node = Corner()
   cs_node.exercise(2)
   time.sleep(2)
   cs_node.run()
