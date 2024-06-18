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
        
        # Odometry subscriber
        sub_odom = message_filters.Subscriber(ODOM_TOPIC, Odometry)
                        
        # Robot pose subscriber
        sub_robot_pose = message_filters.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped)
        
        # Robot Goal subscriber
        rospy.Subscriber(GOAL_TOPIC, MoveBaseActionGoal, self.cb_goal)
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_odom,  
                                                                sub_robot_pose], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)
        
        if SOURCE_FRAME != TARGET_FRAME:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer)

        self.ats.registerCallback(self.get_data)
        
        
    def cb_goal(self, goal: MoveBaseActionGoal):
        """
        Goal callback

        Args:
            goal (MoveBaseActionGoal): robot goal
        """
        self.rg = (goal.goal.target_pose.pose.position.x, goal.goal.target_pose.pose.position.y)
                       

    def get_data(self, odom: Odometry,
                       robot_pose: PoseWithCovarianceStamped):
        """
        Synchronized callback

        Args:
            robot_odom (Odometry): robot odometry
            robot_pose (PoseWithCovarianceStamped): robot pose
        """
        if SOURCE_FRAME != TARGET_FRAME:
            transform = self.tf_buffer.lookup_transform(TARGET_FRAME, SOURCE_FRAME, rospy.Time(), rospy.Duration(0.5))
            original_vector = Vector3Stamped()
            original_vector.header.frame_id = SOURCE_FRAME
            original_vector.vector.x = odom.twist.twist.linear.x
            original_vector.vector.y = odom.twist.twist.linear.y
            original_vector.vector.z = odom.twist.twist.linear.z
            r_v = do_transform_vector3(original_vector, transform)
            
            original_vector = Vector3Stamped()
            original_vector.header.frame_id = SOURCE_FRAME
            original_vector.vector.x = odom.twist.twist.angular.x
            original_vector.vector.y = odom.twist.twist.angular.y
            original_vector.vector.z = odom.twist.twist.angular.z
            r_w = do_transform_vector3(original_vector, transform)
        
            twist = Twist()
            twist.linear.x = r_v.vector.x
            twist.linear.y = r_v.vector.y
            twist.linear.z = r_v.vector.z
            twist.angular.x = r_w.vector.x
            twist.angular.y = r_w.vector.y
            twist.angular.z = r_w.vector.z
        else:
            twist = Twist()
            twist.linear.x = odom.twist.twist.linear.x
            twist.linear.y = odom.twist.twist.linear.y
            twist.linear.z = odom.twist.twist.linear.z
            twist.angular.x = odom.twist.twist.angular.x
            twist.angular.y = odom.twist.twist.angular.y
            twist.angular.z = odom.twist.twist.angular.z
        
               
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