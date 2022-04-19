#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def timekeeper():
    pub = rospy.Publisher('keepalive', String, queue_size=10)
    rospy.init_node('keepalive', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = "%s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        timekeeper()
    except rospy.ROSInterruptException:
        pass
