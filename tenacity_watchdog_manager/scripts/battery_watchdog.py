#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class Tilt:

   def __init__(self):

       self.power_bus=['main_bus_A','main_bus_B']
       self.power_topics=['bus_voltage','bus_power','current']

       for b in self.power_bus:
           power_topic_str="power/"+b+"/"+
           rospy.
   
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

