#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg  Quarternion, Vector3, PoseStamped
from sensor_msgs.msg import Imu

class Incline():

   def __init__(self):
      #init Dict to hold steering state for each servo
      self.imu = Imu()
      self.q = Quaternion()
      warn_x = 0.17
      warn_y = 0.17
      max_x = 0.35
      max_y = 0.35

      rospy.init_node("attitude_watchdog")
      rospy.Subscriber("/imu",Imu,self.imu_read_cb)

   def imu_read_cb(self,data): 

       self.q.x = data.orientation.x 
       self.q.y = data.orientation.y
       self.q.z = data.orientation.z 
       self.q.w = data.orientation.w

       pitch_x = abs(self.q.x)
       pitch_y = abs(self.q.y)
       
       if ((pitch_x > 0.10 ) and (pitch_x < 0.17)):
          rospy.loginfo("Pitch Warning: X = %s",pitch_x)
       if ((pitch_x > 0.17 ) and (pitch_x < 0.35)):
          rospy.loginfo("BANK ANGLE X = %s",pitch_x)
       if ((pitch_x >= 0.35): 
          rospy.loginfo("BANK CRITICAL X = %s",pitch_x)

       if ((pitch_y > 0.10 ) and (pitch_y < 0.17)):
          rospy.loginfo("Pitch Warning: Y = %s",pitch_y)
       if ((pitch_y > 0.17 ) and (pitch_y < 0.35)):
          rospy.loginfo("BANK ANGLE Y = %s",pitch_y)
       if ((pitch_y >= 0.35): 
          rospy.loginfo("BANK CRITICAL Y = %s",pitch_y)
 
   def run(self): 
      rate= rospy.Rate(1)
      while (not rospy.is_shutdown()):
      if ((pitch_x > 0.10 ) and (pitch_x < 0.17)):
         rospy.loginfo("Pitch Warning: X = %s",pitch_x)
      if ((pitch_x > 0.17 ) and (pitch_x < 0.35)):
         rospy.loginfo("BANK ANGLE X = %s",pitch_x)
      if ((pitch_x >= 0.35): 
         rospy.loginfo("BANK CRITICAL X = %s",pitch_x)

      if ((pitch_y > 0.10 ) and (pitch_y < 0.17)):
         rospy.loginfo("Pitch Warning: Y = %s",pitch_y)
      if ((pitch_y > 0.17 ) and (pitch_y < 0.35)):
         rospy.loginfo("BANK ANGLE Y = %s",pitch_y)
      if ((pitch_y >= 0.35): 
         rospy.loginfo("BANK CRITICAL Y = %s",pitch_y)

if __name__ == '__main__':

   i_node = Incline()
   i_node.run()

