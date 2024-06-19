#!/usr/bin/env python

import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from roscausal_msgs.msg import RobotState
import tf
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped, Pose2D, Twist, Point
from move_base_msgs.msg import MoveBaseActionGoal

NODE_NAME = "roscausal_robot"
NODE_RATE = 10 # [Hz]


def get_2DPose(p: PoseWithCovarianceStamped):
    """
    Extracts x, y and theta from pose
    
    Args:
        p (PoseWithCovarianceStamped): pose
    
    Returns:
        tuple: x, y, theta
    """
    x = p.pose.pose.position.x
    y = p.pose.pose.position.y
    
    q = (
        p.pose.pose.orientation.x,
        p.pose.pose.orientation.y,
        p.pose.pose.orientation.z,
        p.pose.pose.orientation.w
    )
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return Pose2D(x, y, yaw)


class RobotStateClass():
    
    def __init__(self) -> None:
        """
        RobotState constructor
        """
        self.rg = None
        
        # Risk publisher     
        self.pub_robot_state = rospy.Publisher('/roscausal/robot', RobotState, queue_size=10)
                                
        # Robot pose subscriber
        sub_robot_pose = rospy.Subscriber(POSE_TOPIC, Odometry, self.get_data)
        
        # Robot Goal subscriber
        rospy.Subscriber(GOAL_TOPIC, MoveBaseActionGoal, self.cb_goal)
                
        # Init synchronizer and assigning a callback 
        # self.ats = message_filters.ApproximateTimeSynchronizer([sub_odom,  
        #                                                         sub_robot_pose], 
        #                                                         queue_size = 10, slop = 0.1,
        #                                                         allow_headerless = True)
    

        # self.ats.registerCallback(self.get_data)
        
        
    def cb_goal(self, goal: MoveBaseActionGoal):
        """
        Goal callback

        Args:
            goal (MoveBaseActionGoal): robot goal
        """
        self.rg = (goal.goal.target_pose.pose.position.x, goal.goal.target_pose.pose.position.y)
                       

    def get_data(self, robot_pose: Odometry):
        """
        Synchronized callback

        Args:
            robot_odom (Odometry): robot odometry
            robot_pose (PoseWithCovarianceStamped): robot pose
        """
        twist = Twist()
        twist.linear.x = robot_pose.twist.twist.linear.x
        twist.linear.y = robot_pose.twist.twist.linear.y
        twist.linear.z = robot_pose.twist.twist.linear.z
        twist.angular.x = robot_pose.twist.twist.angular.x
        twist.angular.y = robot_pose.twist.twist.angular.y
        twist.angular.z = robot_pose.twist.twist.angular.z
        
               
        msg = RobotState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = TARGET_FRAME
                
        msg.pose2D = get_2DPose(robot_pose)
        msg.twist = twist
        if self.rg is not None:
            msg.goal = Point(self.rg[0], self.rg[1], 0)
        else:
            msg.goal = Point(msg.pose2D.x, msg.pose2D.y, 0)
        self.pub_robot_state.publish(msg)
                
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    ODOM_TOPIC = rospy.get_param("~odom_topic", "/mobile_base_controller/odom")
    POSE_TOPIC = rospy.get_param("~pose_topic", "/robot_pose")
    GOAL_TOPIC = rospy.get_param("~goal_topic", "/move_base/goal")
    SOURCE_FRAME = rospy.get_param("~source_frame")
    TARGET_FRAME = rospy.get_param("~target_frame", "map")
    
    r = RobotStateClass()

    while not rospy.is_shutdown():
        rate.sleep()