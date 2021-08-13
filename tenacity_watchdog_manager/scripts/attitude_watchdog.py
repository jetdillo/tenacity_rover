#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion, Vector3, PoseStamped
from sensor_msgs.msg import Imu
from tenacity_watchdog_manager.msg import Tilt

class TiltWatchdog():

   def __init__(self,imu_topic="/imu"):
      self.imu = Imu()
      self.pitch_x = 0.00
      self.pitch_y = 0.00
      self.q = Quaternion()
      self.imu_topic=imu_topic

      #self.warn_x = 0.17
      #self.warn_y = 0.17
      self.max_x = 0.15
      self.max_y = 0.15

      self.max_tilt=0.15

      self.tilt_interval=3
      self.tilt_last=0 
      self.tilt_current=0
      self.alarmed=False

      rospy.init_node("tilt_watchdog")
      rospy.Subscriber(self.imu_topic,Imu,self.imu_read_cb)
      self.tilt_pub=rospy.Publisher('attitude_alarm',Tilt,latch=False,queue_size=2)

   def imu_read_cb(self,data): 

       self.q.x = data.orientation.x 
       self.q.y = data.orientation.y
       self.q.z = data.orientation.z 
       self.q.w = data.orientation.w

       self.pitch_x = abs(self.q.x)
       self.pitch_y = abs(self.q.y)
       
   def run(self): 
      rate= rospy.Rate(10)
      axis=""
      state=""
      angle=0.00

      while (not rospy.is_shutdown()):
           self.tilt_current=time.time()
           tilt=Tilt() 
           if self.pitch_x >= self.max_tilt or self.pitch_y >= self.max_tilt:
              self.alarmed=True
           else:
              self.alarmed=False
              tilt.alarmed=False

           if self.alarmed:
              self.tilt_current=time.time()
              print("%f %f" % (self.tilt_current,self.tilt_last)) 
              if abs(self.tilt_current-self.tilt_last) > self.tilt_interval:
                 print("TILT ALARM %f %f exceeds %f maximum tilt for more than %d seconds" % (self.pitch_x,self.pitch_y,self.max_tilt,self.tilt_interval))
                 tilt.alarmed=True
                 tilt.q=self.q
                 self.tilt_pub.publish(tilt)
                 
           else:
               self.tilt_last = self.tilt_current    
               tilt.alarmed=False
               tilt.q=self.q
               self.tilt_pub.publish(tilt)
                                   
if __name__ == '__main__':

   i_node = TiltWatchdog("gx5/imu/data")
   i_node.run()
