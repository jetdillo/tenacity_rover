#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Range

g_range_ahead = 1 # anything to start
left_range = 0.000
right_range = 0.000
center_range = 0.000 
crit_range = 20.000
z_pose = 0.0000

cur_twist = Twist()

def left_sonar_cb(data):

    left_range = data.range
    print(left_range),

def right_sonar_cb(data):

    right_range = data.range
    print(right_range)

def cur_cv_cb(cv_msg):
    
    z_pose = cv_msg.angular.z

#scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

left_sonar = rospy.Subscriber('/sonar_front_left',Range,left_sonar_cb)
right_sonar = rospy.Subscriber('/sonar_front_right',Range,right_sonar_cb)
cv_state = rospy.Subscriber('/cmd_vel',Twist,cur_cv_cb)


cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
rate = rospy.Rate(10)

t = Twist()

while not rospy.is_shutdown():
   
     if left_range < crit_range:
        #steer(right)
        t.angular.z = z_pose -0.25
#        print("Right Turn:Hazard - %f %f" % (left_range,right_range))
     if right_range < 20: 
        #steer(left)
        t.angular.z = z_pose +0.25
#        print("Left Turn:Hazard - %f %f" % (left_range,right_range))

     if (abs(left_range - right_range) < 10) or (left_range >100 and right_range >100):
        t.angular.z = 0 
#        print("Forward - %f %f %f" % (left_range,right_range,abs(left_range - right_range)))
        #t.linear.x = 0.25
     else:
        if left_range - right_range < -20:
           print("Right Turn:Trim")
           t.angular.z = z_pose -0.1
        if left_range - right_range > 20:
           print("Left Turn:Trim")
           t.angular.z = z_pose +0.1
  
     cmd_vel_pub.publish(t)  
     rate.sleep()
# END ALL
