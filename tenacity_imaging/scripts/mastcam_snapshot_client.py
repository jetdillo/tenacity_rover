#!/usr/bin/env python

import rospy
import actionlib
import argparse
from tenacity_imaging.msg import MastCamSnapshotAction, MastCamSnapshotGoal

def mastcam_snapshot_client(target):
    client = actionlib.SimpleActionClient('mastcam_snapshot_server', MastCamSnapshotAction)
    client.wait_for_server()

    rospy.loginfo("...sending goal")
    client.send_goal(MastCamSnapshotGoal(target),active_cb=cb_active,feedback_cb=cb_feedback,done_cb=cb_done)
    client.wait_for_result()

def cb_active():
    rospy.loginfo("snapshot in progress...")

def cb_done(state,result):
    rospy.loginfo("Action completed with result: %s" % str(result))

def cb_feedback(feedback):
    rospy.loginfo("Feedback on goal:%s" % str(feedback))

if __name__ == '__main__':

    ap = argparse.ArgumentParser(description="Tenacity mastcam snapshot client")
    ap.add_argument('--vista',default='ahead',help="named position")
    ap.add_argument('--pose',default=0.0 ,help="pan/tilt position")
    args=ap.parse_args()

    rospy.loginfo("Starting mastcam_snapshot_client")
    try:
        rospy.init_node('mastcam_snapshot_client')
        if args.vista:
           mastcam_snapshot_client(args.vista)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")

