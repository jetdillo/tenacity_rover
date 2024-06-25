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
      self.pan_rate=0.0
      self.tilt_rate=0.0
      self.pan_newpos=0.0
      self.tilt_newpos=0.0

      self.pan_state=JointState_DM()
      self.tilt_state=JointState_DM()
      rospy.init_node("mastcam_ctl")
      #rospy.Subscriber("/joy", Joy, self.joy_cb)

      rospy.Subscriber('/mastcam_pan_controller/state',JointState_DM,self.pan_state_cb)
      rospy.Subscriber('/mastcam_tilt_controller/state',JointState_DM,self.tilt_state_cb)

      self.pan_sp=rospy.Publisher('/mastcam_pan_controller/command',Float64,queue_size=1)
      self.tilt_sp=rospy.Publisher('/mastcam_tilt_controller/command',Float64,queue_size=1)

   def pan_state_cb(self,data):

       self.pan_state=data 
   
   def tilt_state_cb(self,data):

       self.tilt_state=data 

   def run(self): 
      rate= rospy.Rate(1)
      self.pan_newpos=0.00
      self.tilt_newpos=0.00
      while (not rospy.is_shutdown()):
         if self.cam_mode == 0:
            self.pan_newpos = self.pan_state.current_pos + self.pan_rate
            self.tilt_newpos = self.tilt_state.current_pos + self.tilt_rate

            self.pan_sp.publish(self.pan_newpos)
            self.tilt_sp.publish(self.tilt_newpos)
         else:
            pan_newpos = 0.7878
            tilt_newpos = 0.7878
            self.pan_sp.publish(pan_newpos)
            self.tilt_sp.publish(tilt_newpos)

         time.sleep(1)


#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':

   cs_node = CamCtl()
   cs_node.run()

