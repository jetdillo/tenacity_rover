# coding: utf-8
import time
import RPi.GPIO as GPIO
import rospy
from sensor_msgs.msg import Range


class RasPinger:

   def __init__():
      self.echo_pins=[22,18,17]
      self.ping_names = ["front_center","front_left","front_right"]
      self.return_start=[0.00000,0.00000,0.00000]
      self.return_end=[0.00000,0.00000,0.00000]
      self.ping_count=0

      self.sonar_pub=[0,1,2]

      self.topic_name = "/ultrasonic"
      rospy.init_node("/ultrasound")

      for i in range(0,3):
         self.sonar_pub[i] = rospy.Publisher(topicname+"/"+ping_names[i],sensor_msgs.Range,queue_size=1)
     
      GPIO.setmode(GPIO.BCM)
      for p in self.echo_pins:
         GPIO.setup(p,GPIO.OUT)
         GPIO.output(p,False)

   def measure():

      for p in self.echo_pins:
         GPIO.output(p,True)
         time.sleep(0.00001)
         GPIO.output(p,False)
         ping_times[p]=time.time()

      #wait for all sonars to get the return
      while ping_count < 3: 
         for p in self.echo_pins:
      #Set a sonar pin to input
            GPIO.setup(p,GPIO.IN)

            if GPIO.input(p) == 0:
               self.return_start[p] = time.time()

            if GPIO.input(p) == 1:
              self.return_end[p] = time.time()
              ping_count+= 1 
            
      #All sonars have registered a return
      #reset pins to output and get readt to ping again
      for p in self.echo_pins:
         GPIO.setup(p, GPIO.OUT)
         GPIO.output(p,False)
         
         elapsed = self.return_end[p] - self.return_start[p]
         self.range[p] = elapsed(*34300)/2.0
         time.sleep(0.1)
      
   def pub:
      for i in range(0,3):
         self.sonar_pub[i].publish(self.range[i]) 
         

if __name__ == '__main__':


   
   try:
    while True:
        distance = measure()
        print "  Distance : %.1f cm" % distance
        time.sleep(1)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
