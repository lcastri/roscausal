#!/usr/bin/env python

import rospy
from roscausal_msgs.msg import HumanState, Humans
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovariance, Pose, Point
from darko_messages.darko_perception_msgs.msg import Humans as darkoHumans


NODE_NAME = "roscausal_human"
NODE_RATE = 10 # [Hz]


def get_2DPose(p):
    """
    Extracts x, y and theta from pose
    
    Args:
        p (Pose or PoseWithCovariance): pose
    
    Returns:
        tuple: x, y, theta
    """
    if isinstance(p, PoseWithCovariance):
        x = p.pose.position.x
        y = p.pose.position.y

        q = (
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w
        )
    elif isinstance(p, Pose):
        x = p.position.x
        y = p.position.y

        q = (
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
        )
    else:
        raise ValueError("Unsupported pose type. Expected Pose or PoseWithCovariance.")

    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return Pose2D(x, y, yaw)


class HumanStateClass():
    
    def __init__(self) -> None:
        """
        HumanState constructor
        """
                        
        # Human publisher     
        self.pub_human_state = rospy.Publisher('/roscausal/humans', Humans, queue_size=10)
        
        # Autonomous agents subscriber
        if PEOPLE_TOPIC is not None:
            rospy.Subscriber(PEOPLE_TOPIC, darkoHumans, self.get_auto_data)
               
        
    def get_auto_data(self, humans: darkoHumans):
        """
        Autonomous people callback

        Args:
            people (TrackedPersons): humans
        """
        humans = Humans()
        humans.header = Header()
        humans.header.stamp = rospy.Time.now()
        
        # FIXME: I might not have this info
        pg = rospy.get_param(GOAL_PARAM, None) if GOAL_PARAM is not None else None
        
        for person in humans.humans:
            state = get_2DPose(person.centroid)
        
            # msg
            msg = HumanState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            # FIXME: is the source frame map? If not, I need to transform it into the map frame
            msg.header.frame_id = TARGET_FRAME
            msg.id = int(person.id)
            msg.pose2D = Pose2D(state.x, state.y, state.theta)
            
            twist = Twist()
            twist.linear.x = person.velocity.twist.linear.x
            twist.linear.y = person.velocity.twist.linear.y
            twist.linear.z = person.velocity.twist.linear.z
            twist.angular.x = person.velocity.twist.angular.x
            twist.angular.y = person.velocity.twist.angular.y
            twist.angular.z = person.velocity.twist.angular.z        
            msg.twist = twist
            
            # FIXME: I might not have this info
            if pg is not None:
                msg.goal = Point(pg[0], pg[1], 0)
            else:
                msg.goal = Point(msg.pose2D.x, msg.pose2D.y, 0)
            
            humans.humans.append(msg)
            
        H.pub_human_state.publish(humans)
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    PEOPLE_TOPIC = rospy.get_param("~people_topic", "")
    PEOPLE_TOPIC = None if PEOPLE_TOPIC == "" else PEOPLE_TOPIC
    
    GOAL_PARAM = rospy.get_param("~goal_param", "")
    GOAL_PARAM = None if GOAL_PARAM == "" else GOAL_PARAM
    
    SOURCE_FRAME = rospy.get_param("~source_frame")
    TARGET_FRAME = rospy.get_param("~target_frame", "map")
    
    H = HumanStateClass()

    rospy.spin()
