#!/usr/bin/env python

import rospy
from roscausal_msgs.msg import HumanState, Humans
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovariance, TwistWithCovariance, Pose, Point
from spencer_tracking_msgs.msg import TrackedPersons
from shapely.geometry import Point, Polygon
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


NODE_NAME = "roscausal_human"
NODE_RATE = 10 # [Hz]
MAP_BOUNDARIES = [] # add points
MAP_POLY = Polygon(MAP_BOUNDARIES)



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
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
            
            
    def get_data(self, people: TrackedPersons):
        """
        people callback

        Args:
            people (TrackedPersons): people
        """
        rospy.logerr("CIAO")
        
        humans = Humans()
        humans.header = Header()
        humans.header.stamp = rospy.Time.now()
        
        # FIXME: I might not have this info
        pg = rospy.get_param(GOAL_PARAM, None) if GOAL_PARAM is not None else None
        
        for person in people.tracks:
            
            trans = self.tf_buffer.lookup_transform(TARGET_FRAME, humans.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(person.pose, trans)

            transformed_twist = TwistWithCovariance()
            transformed_twist.header.stamp = rospy.Time.now()
            transformed_twist.header.frame_id = TARGET_FRAME

            # Transform linear velocity
            linear_velocity = tf2_geometry_msgs.Vector3Stamped()
            linear_velocity.vector = person.twist.twist.linear
            linear_velocity.header = person.header
            transformed_linear_velocity = tf2_geometry_msgs.do_transform_vector3(linear_velocity, trans)

            # Transform angular velocity
            angular_velocity = tf2_geometry_msgs.Vector3Stamped()
            angular_velocity.vector = person.twist.twist.angular
            angular_velocity.header = person.header
            transformed_angular_velocity = tf2_geometry_msgs.do_transform_vector3(angular_velocity, trans)

            transformed_twist.twist.linear = transformed_linear_velocity.vector
            transformed_twist.twist.angular = transformed_angular_velocity.vector
            transformed_twist.covariance = twist.twist.covariance
            
            
            state = get_2DPose(transformed_pose)
        
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
            
            humans.humans.append(msg)
            
        self.pub_human_state.publish(humans)

               
        
    # def get_data(self, humans: TrackedPersons):
    #     """
    #     Autonomous people callback

    #     Args:
    #         people (TrackedPersons): humans
    #     """
    #     humans = Humans()
    #     humans.header = Header()
    #     humans.header.stamp = rospy.Time.now()
        
    #     # FIXME: I might not have this info
    #     pg = rospy.get_param(GOAL_PARAM, None) if GOAL_PARAM is not None else None
        
    #     for person in humans.humans:
    #         state = get_2DPose(person.centroid)
    #         person_pos = Point(state.x, state.y)

    #         if MAP_POLY.contains(person_pos): # this is useful for checking if the person is within a certain area
        
    #             # msg
    #             msg = HumanState()
    #             msg.header = Header()
    #             msg.header.stamp = rospy.Time.now()
    #             # FIXME: is the source frame map? If not, I need to transform it into the map frame
    #             msg.header.frame_id = TARGET_FRAME
    #             msg.id = int(person.id)
    #             msg.pose2D = Pose2D(state.x, state.y, state.theta)
                
    #             twist = Twist()
    #             twist.linear.x = person.velocity.twist.linear.x
    #             twist.linear.y = person.velocity.twist.linear.y
    #             twist.linear.z = person.velocity.twist.linear.z
    #             twist.angular.x = person.velocity.twist.angular.x
    #             twist.angular.y = person.velocity.twist.angular.y
    #             twist.angular.z = person.velocity.twist.angular.z        
    #             msg.twist = twist
                
    #             # FIXME: I might not have this info
    #             if pg is not None:
    #                 msg.goal = Point(pg[0], pg[1], 0)
    #             else:
    #                 msg.goal = Point(msg.pose2D.x, msg.pose2D.y, 0)
                
    #             humans.humans.append(msg)
            
    #     self.pub_human_state.publish(humans)
        

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
