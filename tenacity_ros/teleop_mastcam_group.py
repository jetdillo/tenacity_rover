#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamixel_msgs.msg import JointState as JointState_DM
from sensor_msgs.msg import JointState as JointState_SM

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner caming. 

cam_min=205
cam_max=815

cam_state={}

class CamCtl:

   def __init__(self):
      #init Dict to hold caming state for each servo
      self.cam_mode=0
      self.cam_state={}
      self.pan_rate=0.0
      self.tilt_rate=0.0
      self.jss = JointState_SM()
      rospy.init_node("mastcam_ctl")
      rospy.Subscriber("/joy", Joy, self.joy_cb)

      self.jss.name=self.cam_controllers=['mastcam_pan','mastcam_tilt']

      self.jss.position=[0.0,0.0,0.0,0.0]
      self.jss.velocity=[0.0,0.0,0.0,0.0]

      for s in self.cam_controllers:
         self.cam_state[s]=0.0

      for s in self.cam_controllers:
         cam_topic_str=s+"_controller/state"
         print cam_topic_str
         rospy.Subscriber(cam_topic_str,JointState_DM,self.cam_state_cb)

      self.pan_sp=rospy.Publisher('/mastcam_pan_controller/command',Float64,queue_size=1)
      self.tilt_sp=rospy.Publisher('/mastcam_tilt_controller/command',Float64,queue_size=1)

      #self.js_repub=rospy.Publisher('joint_states',JointState_SM,queue_size=1)

   def joy_cb(self,data):

      #self.pan_rate=data.axes[3]
      #self.tilt_rate=data.axes[4]
   
      print(data.buttons)
      if data.buttons[7] == 1:
         self.cam_mode=1
         print("Setting home mode for mastcam")
     
      if data.buttons[0] == 1:
         print("tilt down")
         self.tilt_rate=-0.1
     
      if data.buttons[3] == 1:
         print("tilt up")
         self.tilt_rate=0.1

      if data.buttons[2] == 1:
         print("pan left")
         self.pan_rate=0.1

      if data.buttons[1] == 1:
         print("pan right")
         self.pan_rate=-0.1
   
   def cam_state_cb(self,data):

      jsp=[]
      js=JointState_DM()
      js.name=data.name
      js.motor_ids=data.motor_ids
      js.current_pos=data.current_pos
      self.cam_state[js.name]=js.current_pos

      for jp in self.jss.name:
         jsp.append(self.cam_state[jp])
      
      for j in range(0,len(jsp)):
         self.jss.position[j]=jsp[j]

   def run(self): 
      rate= rospy.Rate(1)
      pan_newpos=0.00
      tilt_newpos=0.00
      while (not rospy.is_shutdown()):
       
         if self.cam_mode == 0:
            pan_newpos = self.cam_state['mastcam_pan'] + self.pan_rate
            tilt_newpos = self.cam_state['mastcam_tilt'] + self.tilt_rate
         
            self.pan_sp.publish(pan_newpos)
            self.tilt_sp.publish(tilt_newpos)
         else:
            pan_newpos = 0.7878
            tilt_newpos = 0.7878
            self.pan_sp.publish(pan_newpos)
            self.tilt_sp.publish(tilt_newpos)


#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 steering IDs: 1,2,5,6
#  One Pan ID: ?
#  One Tilt ID: 7
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   cs_node = CamCtl()
   cs_node.run()

