#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg  Quarternion, Vector3, PoseStamped
from sensor_msgs.msg import BatteryState

class Battery():

   def __init__(self,battname):
      #init Dict to hold steering state for each servo
      self.bs = BatteryState()
      self.pack=battname
      self.topic=battname+"/battery/info" 

      rospy.init_node("battstat")
      rospy.Subscriber(self.topic,BatteryState,self.batt_state_read_cb)

#Populate useful bits of the BatteryState message
   def batt_state_read_cb(self,data): 
   
      bs.voltage=data.voltage
      bs.percentage=data.percentage
      bs.location =data.location
      
if __name__ == '__main__':


   battStatt = Battery("tenacity")
