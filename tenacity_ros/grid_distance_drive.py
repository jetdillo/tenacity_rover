#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64,Bool,Int16,Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

#Generic steering interface for non-teleop nodes to control the Dynamixels that control the corner steering.
#Reads off /cmd_vel like a normal control interface and publishes those values to 

class DistanceDrive:

   def __init__(self,drive_distance):
      #init Dict to hold steering state for each servo
      self.twist = Twist()
      self.throttle=0
      self.max_speed=0.5
      self.left_ticks=0
      self.right_ticks=0
      self.ticks_per_rev=823
      self.ticks_per_meter=2025
      self.grid_size=self.ticks_per_meter
     
      self.drive_length=drive_distance*self.ticks_per_meter

      rospy.init_node("distance_drive")

      rospy.Subscriber('cmd_vel',Twist,self.cmd_vel_cb)
      rospy.Subscriber('left_ticks',Int32,self.left_ticks_cb)
      rospy.Subscriber('right_ticks',Int32,self.right_ticks_cb)

      self.drive=rospy.Publisher('cmd_vel',Twist,queue_size=10)

   def cmd_vel_cb(self,data):
      self.twist.linear.x = data.linear.x
      self.twist.angular.z = data.angular.z
      
   def left_ticks_cb(self,data):
      self.left_ticks=data
   
   def right_ticks_cb(self,data):
      self.right_ticks=data

   def run(self): 
      t=Twist() 
      rate= rospy.Rate(1)
      drive_dest=self.left_ticks+drive_length
      while (not rospy.is_shutdown()):

          while ( self.left_ticks < drive_dest):
             t.linear.x=0.5
             self.drive.publish(t)
          t=Twist()
          self.drive.publish(t)
                      
if __name__ == '__main__':

   d = DistanceDrive(1)
   d.run()
