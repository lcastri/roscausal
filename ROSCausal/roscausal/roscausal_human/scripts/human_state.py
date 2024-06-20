#!/usr/bin/env python

import rospy
from roscausal_msgs.msg import HumanState, Humans
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovariance, TwistWithCovariance, Pose, Point
from spencer_tracking_msgs.msg import TrackedPersons
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from std_msgs.msg import String


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
        
        rospy.Subscriber(PEOPLE_TOPIC, TrackedPersons, self.get_data)
        
        rospy.Subscriber('/roscausal/goal', String, self.get_goal)
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.goal = None
            
            
    def get_goal(self, g: String):
        gg = str(g.data)
        if gg != '':
            self.goal = gg.split("_")
            self.goal = [float(i) for i in self.goal]
        else:
            self.goal = None
            
                     
    def get_data(self, people: TrackedPersons):
        """
        people callback

        Args:
            people (TrackedPersons): people
        """        
        humans = Humans()
        humans.header = Header()
        humans.header.stamp = rospy.Time.now()
                
        for person in people.tracks:
            
            trans = self.tf_buffer.lookup_transform(TARGET_FRAME, people.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(person.pose, trans)

            transformed_twist = TwistWithCovariance()

            # Transform linear velocity
            linear_velocity = tf2_geometry_msgs.Vector3Stamped()
            linear_velocity.vector = person.twist.twist.linear
            transformed_linear_velocity = tf2_geometry_msgs.do_transform_vector3(linear_velocity, trans)

            # Transform angular velocity
            angular_velocity = tf2_geometry_msgs.Vector3Stamped()
            angular_velocity.vector = person.twist.twist.angular
            transformed_angular_velocity = tf2_geometry_msgs.do_transform_vector3(angular_velocity, trans)

            transformed_twist.twist.linear = transformed_linear_velocity.vector
            transformed_twist.twist.angular = transformed_angular_velocity.vector
            
            
            state = get_2DPose(transformed_pose.pose)
        
            # msg
            msg = HumanState()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = TARGET_FRAME
            msg.id = 1000 # NOTE: this is to ensure that the tracked person has a specific ID 
            # msg.id = int(person.track_id)
            msg.pose2D = Pose2D(state.x, state.y, state.theta)
            
            twist = Twist()
            twist.linear.x = transformed_twist.twist.linear.x
            twist.linear.y = transformed_twist.twist.linear.y
            twist.linear.z = transformed_twist.twist.linear.z
            twist.angular.x = transformed_twist.twist.angular.x
            twist.angular.y = transformed_twist.twist.angular.y
            twist.angular.z = transformed_twist.twist.angular.z        
            msg.twist = twist
            
            if self.goal is not None:
                msg.goal = Point(self.goal[0], self.goal[1], 0)
            else:
                msg.goal = Point(-1000, -1000, 0) # NOTE: THIS IS A TRICK
            
            humans.humans.append(msg)
            
            
            break # NOTE: this is to ensure that there is only one person in the scene 
            
        self.pub_human_state.publish(humans)
       

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    PEOPLE_TOPIC = rospy.get_param("~people_topic", "")
    PEOPLE_TOPIC = None if PEOPLE_TOPIC == "" else PEOPLE_TOPIC
    
    GOAL_PARAM = rospy.get_param("~goal_param", "")
    GOAL_PARAM = None if GOAL_PARAM == "" else GOAL_PARAM
    
    TARGET_FRAME = rospy.get_param("~target_frame", "map")
    
    H = HumanStateClass()

    rospy.spin()
