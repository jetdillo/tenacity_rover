#!/usr/bin/env python
import rospy
import time
from control_msgs.msg import FollowJointTrajectoryFeedback
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy,JointState
from trajectory_msgs.msg import JointTrajectory

#subscribes to /cmd_vel and processes angular_vel data out to dynamixels for corner steering. 

steer_min=102
steer_max=915
steering_joints=[]
steering_positions=[]
steer_vel=0.0

def cmdvel_cb(data):
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
   steering_joints=data.joint_names
   steering_positions=data.actual.positions
   #steer_min=data.dynamixel_msg.min
   #steer_max=data.dynamixel_msg.max

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
   rospy.Subscriber("/cmd_vel",Twist,cmdvel_cb)
   rospy.Subscriber("/f_arm_controller/state",FollowJointTrajectoryFeedback,steering_state_cb)
 
    
   steering_state_publisher=rospy.Publisher('/f_arm_controller/command',JointTrajectory,queue_size=10) 
   #right_rear_sp=rospy.Publisher('rear_right_controller/command',Float64,queue_size=10)

   exercise_positions=[0.0,0.785,1.5757,0.0,-0.785,-1.5757,0.0]
   for e in exercise_positions:
      jt=JointTrajectory()
      for j in range(0,(len(steering_joints))):
          print e
          jt.names[j]=steering_joints[j]
          jt.positions[j]=e
      steering_state_publisher.publish(jt)
      time.sleep(1)
   
#  subscribe to Dynamixel channel to get list of servos
#  There should be 4 IDs: 1,2,5,6
#  We need to error out of there are less or if the IDs are different
#  Is there a way to verify a topic exists ?

if __name__ == '__main__':
   start()
