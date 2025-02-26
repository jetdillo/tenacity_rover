#!/usr/bin/env python

import rospy
import actionlib
from tenacity_imaging.msg import PanoramaScanAction, PanoramaScanGoal

def test_panorama_scan():
    rospy.init_node('test_panorama_scan')
    client = actionlib.SimpleActionClient('panorama_scan', PanoramaScanAction)
    client.wait_for_server()

    goal = PanoramaScanGoal()
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Scan directory: %s" , result.scan_directory)

if __name__ == '__main__':
    test_panorama_scan()
