#!/usr/bin/env python

import rospy
from roscausal_msgs.msg import HumanState, Humans
from pedsim_msgs.msg import TrackedPersons, AgentStates
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
        
        self.humans = list()
                
        # Human publisher     
        self.pub_human_state = rospy.Publisher('/roscausal/human', Humans, queue_size=10)
        
        # Teleop agent subscriber
        rospy.Subscriber(TELEOP_PEOPLE_TOPIC, TrackedPersons, self.get_teleop_data)
        
        # Autonomous agents subscriber
        rospy.Subscriber(AUTO_PEOPLE_TOPIC, AgentStates, self.get_auto_data)
        
        
    def get_teleop_data(self, people: TrackedPersons):
        """
        Teleop people callback

        Args:
            people (TrackedPersons): people
        """
        pg = rospy.get_param(GOAL_PARAM, None)
        
        person = people.tracks[0]
        state = get_2DPose(person.pose)
    
        # msg
        msg = HumanState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = TARGET_FRAME
        msg.id = int(person.track_id)
        msg.pose2D = Pose2D(state.x, state.y, state.theta)
        
        twist = Twist()
        twist.linear.x = person.twist.twist.linear.x
        twist.linear.y = person.twist.twist.linear.y
        twist.linear.z = person.twist.twist.linear.z
        twist.angular.x = person.twist.twist.angular.x
        twist.angular.y = person.twist.twist.angular.y
        twist.angular.z = person.twist.twist.angular.z        
        msg.twist = twist
        
        if pg is not None:
            msg.goal = Point(pg[0], pg[1], 0)
        else:
            msg.goal = Point(msg.pose2D.x, msg.pose2D.y, 0)
        
        # Check if the agent already exists in self.humans. If so, remove it before appending the new msg
        for existing_msg in self.humans:
            if existing_msg.id == msg.id:
                self.humans.remove(existing_msg)
                break

        self.humans.append(msg)
        
        
    def get_auto_data(self, people: AgentStates):
        """
        Autonomous people callback

        Args:
            people (TrackedPersons): people
        """
        for person in people.agent_states:
            state = get_2DPose(person.pose)
        
            # msg
            msg = HumanState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = TARGET_FRAME
            msg.id = int(person.id)
            msg.pose2D = Pose2D(state.x, state.y, state.theta)
            
            twist = Twist()
            twist.linear.x = person.twist.linear.x
            twist.linear.y = person.twist.linear.y
            twist.linear.z = person.twist.linear.z
            twist.angular.x = person.twist.angular.x
            twist.angular.y = person.twist.angular.y
            twist.angular.z = person.twist.angular.z        
            msg.twist = twist
            
            msg.goal = person.goal
            
            # Check if the agent already exists in self.humans. If so, remove it before appending the new msg
            for existing_msg in self.humans:
                if existing_msg.id == msg.id:
                    self.humans.remove(existing_msg)
                    break

            self.humans.append(msg)
                

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    AUTO_PEOPLE_TOPIC = rospy.get_param("~auto_people_topic", "/pedsim_simulator/simulated_agents")
    TELEOP_PEOPLE_TOPIC = rospy.get_param("~teleop_people_topic", "/ped/control/teleop_persons")
    GOAL_PARAM = rospy.get_param("~goal_param", "/hri/human_goal")
    SOURCE_FRAME = rospy.get_param("~source_frame")
    TARGET_FRAME = rospy.get_param("~target_frame", "map")
    
    H = HumanStateClass()

    while not rospy.is_shutdown():
        humans = Humans()
        humans.humans = H.humans
        H.pub_human_state.publish(humans)
        
        # FIXME: not needed
        H.humans = list()
        
        rate.sleep()