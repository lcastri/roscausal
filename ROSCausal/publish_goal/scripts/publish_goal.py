#!/usr/bin/env python

import rospy
from roscausal_msgs.msg import Humans
import math
from std_msgs.msg import String


NODE_NAME = "publish_goal"
NODE_RATE = 10 # [Hz]
DIST_THRES = 0.5
GOALS = [] # FIXME: update the goal list


class PublishGoalClass():
    
    def __init__(self) -> None:
        """
        HumanState constructor
        """
                        
        # Humans subscriber    
        rospy.Subscriber('/roscausal/humans', Humans, self.get_data)
        self.pub_goal = rospy.Publisher('/roscausal/goal', String, queue_size=10)

            
    def get_data(self, humans: Humans):
        """
        humans callback

        Args:
            humans (Humans): humans
        """
        goal = ''
        for human in humans.humans:
            human.pose2D.x
            human.pose2D.y
            
            for goal in GOALS:
                if math.sqrt((human.pose2D.x - goal[0])**2 + (human.pose2D.y - goal[1])**2) <= DIST_THRES:
                    goal = str(goal[0]) + "_" + str(goal[1])
                    break
            break
        msg = String()
        msg.data = goal
        self.pub_goal.publish(msg.data)

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
        
    H = PublishGoalClass()

    rospy.spin()
