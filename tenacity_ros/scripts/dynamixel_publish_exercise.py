#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy,JointState


#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 

steer_min=102
steer_max=915

def steering_cb(data):
   t  = Twist()
   t.angular.z = data.angular.z

   #Magic happens here. 
   #Get Joint states from JointStatePublisher 
     
   #publish_steering(steer_angle)
   
#def publish_steering(data):
#
#   #Publish steer_angle to the front servos
#   #Publish the mirror of steer_angle to the rear ones
#
def steering_state_cb(data):
   steer_min=data.dynamixel_msg.min
   steer_max=data.dynamixel_msg.max

def motor_state_cb(data):
   print(data.name)

#def twist_to_radian(zdata):
#
#   #if zdata >=0:
#   
#   if zdata < 0:
      
# Intializes everything
def start():
   rospy.init_node('corner_steering')

   steer_controllers=['front_left','front_right','rear_left','rear_right']
   rospy.Subscriber("/cmd_vel",Twist,steering_cb)
   for s in steer_controllers: 
      steer_topic_str=s+"_controller/state" 
      print steer_topic_str
      rospy.Subscriber(steer_topic_str,JointState,steering_state_cb)
   #rospy.Subscriber("/motor_states/pan_tilt_port",motor_state_cb)
   
   left_front_sp=rospy.Publisher('front_left_controller/command',Float64,queue_size=10)
   right_front_sp=rospy.Publisher('front_right_controller/command',Float64,queue_size=10)

   left_rear_sp=rospy.Publisher('rear_left_controller/command',Float64,queue_size=10)
   right_rear_sp=rospy.Publisher('rear_right_controller/command',Float64,queue_size=10)

   exercise_positions=[0.0,0.785,1.5757,0.0,-0.785,-1.5757,0.0]
   for e in exercise_positions:
      print e
   #   js = JointState()
   #   js.name=""
   #   js.position=e
   #   js.velocity=0.393
      left_front_sp.publish(e)
      right_front_sp.publish(e)
      left_rear_sp.publish(e)
      right_rear_sp.publish(e)
      
      time.sleep(1)
   
#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':
   start()
