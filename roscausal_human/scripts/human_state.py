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

# DIST_THRES = float(rospy.get_param("/hri/safe_distance", default = 5.0))
# INC_SIZE = float(rospy.get_param("/hri/obs_size", default = 2.5))

# def wrap(angle, lower_bound, upper_bound):
#     """
#     Wrap an angle to be within the specified bounds.

#     Args:
#         angle (float): The angle to be wrapped.
#         lower_bound (float): The lower bound for the angle.
#         upper_bound (float): The upper bound for the angle.

#     Returns:
#         float: The wrapped angle.
#     """
#     range_width = upper_bound - lower_bound
#     wrapped_angle = (angle - lower_bound) % range_width + lower_bound

#     return wrapped_angle


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


# def compute_risk(A: Point, obs: Point, Av: Point, obsv: Point, noise):
#     risk = noise
#     collision = False
    
#     # Calculate relative velocity vector
#     Vrel = Point(obsv.x - Av.x, obsv.y - Av.y)
           
#     # Compute the slope of the line AB and line PAB ⊥ AB
#     slope_AB = (obs.y - A.y) / (obs.x - A.x)  # Rise over run
#     slope_PAB = -1 / slope_AB
#     # Choose distances to the left and right of point B

#     # Calculate coordinates for two points along the perpendicular line
#     # Calculate the change in x and y based on the fixed horizontal distance
#     delta_x = INC_SIZE / (1 + slope_PAB ** 2) ** 0.5
#     delta_y = delta_x * slope_PAB
#     # Calculate coordinates for two points along the perpendicular line
#     left = Point(obs.x - delta_x, obs.y - delta_y)
#     right = Point(obs.x + delta_x, obs.y + delta_y)
#     # Cone
#     cone_origin = Point(A.x, A.y)               
#     cone = Polygon([cone_origin, left, right])
    
#     P = Point(cone_origin.x + Av.x, cone_origin.y + Av.y)
#     collision = P.within(cone) and A.distance(obs) < DIST_THRES
#     if collision:
#         time_collision_measure = A.distance(obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
#         steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
#         risk = risk + 1/time_collision_measure + steering_effort_measure
                
#     # return math.exp(risk)
#     return risk, collision, cone_origin, left, right


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
        pg = rospy.get_param(GOAL_SRV, None)
        
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
        
        
    # def heading(self, g):
    #     """
    #     heading angle
        
    #     Args:
    #         g (Point): target point
            
    #     Returns:
    #         float: heading angle
    #     """
        
    #     angle = wrap(2*np.pi - wrap(math.atan2(g[1]-self.y, g[0]-self.x) - wrap(self.theta, 0, 2*np.pi), 0, 2*np.pi), -np.pi, np.pi)
    #     return angle
    
    
    # def risk(self, robot: RobotState, noise):
    #     """
    #     Risk

    #     Args:
    #         robot (RobotState): robot state
    #         people (TrackedPersons): tracked people
    #     """
    #     # Robot 2D pose (x, y, theta) and velocity
    #     r_x = robot.pose2D.x
    #     r_y = robot.pose2D.y
    #     r_v = robot.twist.linear

    #     # Human 2D pose (x, y, theta) and velocity       
    #     A = Point(self.x, self.y)
    #     Av = Point(self.v.x, self.v.y)
    #     obs = Point(r_x, r_y)
    #     obsv = Point(r_v.x, r_v.y)
        
    #     return compute_risk(A, obs, Av, obsv, noise)
        
                
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    PEOPLE_TOPIC = rospy.get_param("~people_topic", "/ped/control/teleop_persons")
    GOAL_SRV = rospy.get_param("~goal_srv", "/hri/human_goal")
    SOURCE_FRAME = rospy.get_param("~source_frame")
    TARGET_FRAME = rospy.get_param("~target_frame", "map")
    
    r = HumanStateClass()

    while not rospy.is_shutdown():
        rate.sleep()