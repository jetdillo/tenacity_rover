#!/usr/bin/env python

import rospy
import actionlib
from tenacity_imaging.msg import MastCamSnapshotAction, MastCamSnapshotGoal

def mastcam_snapshot_client():
    rospy.init_node('mastcam_snapshot_client')
    client = actionlib.SimpleActionClient('mastcam_snapshot', MastCamSnapshotAction)
    client.wait_for_server()

    goal = MastCamSnapshotGoal()
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Scan directory: %s" , result.scan_directory)

if __name__ == '__main__':
    mastcam_snapshot_client()
