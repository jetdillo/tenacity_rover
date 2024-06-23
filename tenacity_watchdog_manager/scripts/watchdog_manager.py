#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist, Quaternion, Vector3, PoseStamped
from sensor_msgs.msg import Imu,Range
from tenacity_watchdog_manager.msg import Tilt
from tenacity_watchdog_manager.msg import Statboard

class WatchdogManager():

   def __init__(self):

      self.alarms=[False,False,False]
      self.t = Tilt()
      rospy.init_node('watchdog_manager')
      self.alarm_pub=rospy.Publisher('watchboard',Statboard,latch=True,queue_size=2)
      
      self.cliffsub = rospy.Subscriber('/cliff_state',Bool,cliff_state_cb)
      self.attsub = rospy.Subscriber('/attitude_alarm',Tilt,self.att_state_cb)
      self.battsub = rospy.Subscriber('/batt_state',Bool,batt_state_cb)

   def cliff_read_cb(self,data):
     
       self.alarms[0]=data.data

   def att_state_cb(self,data):

       self.t = data       
       self.alarms[1]=self.t.alarmed
       
       if self.t.alarmed:
          print("***Tilt Alarm Triggered***")
          print("Orientation: %s" % self.t.q)

   def batt_state_cb(self,data):

       self.alarms[2]=data.data

   def run(self): 
      rate= rospy.Rate(10)
      while (not rospy.is_shutdown()):
         sb = Statboard()
         sb=self.alarms
         self.alarm_pub.publish(sb)


if __name__ == '__main__':

   #cliff_node = Cliff("/sonar_front_center")
   #cliff_node.run()

   wm = WatchdogManager()
   wm.run()
