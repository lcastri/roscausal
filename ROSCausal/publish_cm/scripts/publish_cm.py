#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


NODE_NAME = "publish_cm"
NODE_RATE = 10 # [Hz]


def publish_message(event):
    rospy.logerr("CIAO")
    pub_tsdag = rospy.Publisher('/roscausal/tsdag', Image, queue_size=10)
    bridge = CvBridge()
    # Publish tsDAG
    img = cv2.imread(CM_PATH)
    img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    pub_tsdag.publish(img_msg)


if __name__ == '__main__':
    rospy.logerr("HERE")
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    CM_PATH = str(rospy.get_param("~cm_path", default = ''))
    
    rospy.Timer(rospy.Duration(10), publish_message)
    
    rospy.spin()
