#!/usr/bin/env python

import rospy
from roscausal_msgs.msg import HumanState
from pedsim_msgs.msg import TrackedPersons
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovariance
from shapely.geometry import *


NODE_NAME = "roscausal_human"
NODE_RATE = 10 # [Hz]


def get_2DPose(p: PoseWithCovariance):
    """
    Extracts x, y and theta from pose
    
    Args:
        p (PoseWithCovarianceStamped): pose
    
    Returns:
        tuple: x, y, theta
    """
    x = p.pose.position.x
    y = p.pose.position.y
    
    q = (
        p.pose.orientation.x,
        p.pose.orientation.y,
        p.pose.orientation.z,
        p.pose.orientation.w
    )
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return Pose2D(x, y, yaw)


class HumanStateClass():
    
    def __init__(self) -> None:
        """
        HumanState constructor
        """
        self.x = None
        self.y = None
        self.theta = None
        self.v = None
        self.w = None
        
        self.robot = None
        
        # Risk publisher     
        self.pub_human_state = rospy.Publisher('/roscausal/human', HumanState, queue_size=10)
        
        # TrackedPersons subscriber
        rospy.Subscriber("/ped/control/teleop_persons", TrackedPersons, self.get_data)
        
        
    def get_data(self, people: TrackedPersons):
        """
        Synchronized callback

        Args:
            people (TrackedPersons): people
        """
        pg = rospy.get_param(GOAL_PARAM, None)
        
        person = people.tracks[0]
        state = get_2DPose(person.pose)
        self.x = state.x
        self.y = state.y
        self.theta = state.theta
        self.v = person.twist.twist.linear
        self.w = person.twist.twist.angular    
    
        # msg
        msg = HumanState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = TARGET_FRAME
        msg.pose2D = Pose2D(self.x, self.y, self.theta)
        
        twist = Twist()
        twist.linear.x = self.v.x
        twist.linear.y = self.v.y
        twist.linear.z = self.v.z
        twist.angular.x = self.w.x
        twist.angular.y = self.w.y
        twist.angular.z = self.w.z        
        msg.twist = twist
        
        if pg is not None:
            msg.goal = Point(pg[0], pg[1], 0)
        else:
            msg.goal = Point(msg.pose2D.x, msg.pose2D.y, 0)
        
        self.pub_human_state.publish(msg)               
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    PEOPLE_TOPIC = rospy.get_param("~people_topic", "/ped/control/teleop_persons")
    GOAL_PARAM = rospy.get_param("~goal_param", "/hri/human_goal")
    SOURCE_FRAME = rospy.get_param("~source_frame")
    TARGET_FRAME = rospy.get_param("~target_frame", "map")
    
    r = HumanStateClass()

    while not rospy.is_shutdown():
        rate.sleep()