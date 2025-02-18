import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState as JointState_SM
from dynamixel_msgs.msg import JointState as JointState_DM
from geometry_msgs.msg import Twist
from depthai_ros_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2

rospy.init_node('image_listener', anonymous=True)
    bridge = CvBridge()
    image_topic = "/camera/rgb/image_raw" # Replace with your image topic

def image_callback(data):
   try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
   except CvBridgeError as e:
            print(e)

   # Process the cv_image (e.g., display, save, analyze)
