#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge


NODE_NAME = "publish_cm"
NODE_RATE = 10 # [Hz]

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    CM_PATH = str(rospy.get_param("~cm_path", default = ''))

    time = rospy.get_time()
    pub_tsdag = rospy.Publisher('/roscausal/tsdag', String, queue_size=10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        
        while rospy.get_time() - time <= 60:
            
            # Publish tsDAG
            img = cv2.imread(CM_PATH)
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            pub_tsdag.publish(img_msg)
            time = rospy.get_time()
        
            rate.sleep()
