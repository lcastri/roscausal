#!/usr/bin/env python

from datetime import datetime
import rospy
import pandas as pd
import os
import message_filters
from roscausal_msgs.msg import HumanState
from roscausal_msgs.msg import RobotState


NODE_NAME = "roscausal_data"
NODE_RATE = 10 # [Hz]
TS_LENGTH = float(rospy.get_param("~ts_length", default = 150)) # [s]
DATA_DIR = str(rospy.get_param("~data_dir")) + '/data_pool'
DT = float(rospy.get_param("~dt", default = 0.1))
SUBSAMPLING = bool(rospy.get_param("~subsampling", default = False))
ID_FORMAT = str(rospy.get_param("~id_format", default = '%Y%m%d_%H%M%S'))
CSV_PREFIX = str(rospy.get_param("~csv_prefix", default = 'data_'))


class DataCollector():

    def __init__(self) -> None:
        """
        DataCollector constructor
        """
        self.raw = None
        self.time_init = None
                        
        # Robot  subscriber
        sub_robot = message_filters.Subscriber("/roscausal/robot", RobotState)
        
        # Person subscriber
        sub_people = message_filters.Subscriber('/roscausal/human', HumanState)
                        
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_robot, 
                                                                sub_people], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.cb_handle_data)
                    

                              
    def cb_handle_data(self, robot: RobotState, human: HumanState):
        """
        Synchronized callback

        Args:
            robot (RobotState): robot state
            human (HumanState): tracked person
        """
        time_now = robot.header.stamp.to_sec()
        if self.time_init is None or (time_now - self.time_init >= TS_LENGTH):
            
            # Save currect dataframe
            if self.df is not None:
                # this is to make sure that the dataframe contains data with time in ascending order.
                # with ApproximateTimeSynchronizer might not be always true
                self.raw.sort_values(by = 'time', ascending = True, inplace = True, ignore_index = True)
                
                # subsample your dataset
                if SUBSAMPLING: self.raw = self.subsampling(self.raw, DT)
                timestamp_str = datetime.now().strftime(ID_FORMAT)

                rospy.logwarn("CSV file saved: " + CSV_PREFIX + timestamp_str + '.csv')
                self.raw.to_csv(DATA_DIR + '/' + CSV_PREFIX + timestamp_str + '.csv', index=False)

            
            # Init dataframe
            columns = ['time', 'r_{gx}', 'r_{gy}', 'r_x', 'r_y', 'r_{\theta}', 'r_v', 'r_{\omega}', 'h_{gx}', 'h_{gy}', 'h_x', 'h_y', 'h_{\theta}', 'h_v', 'h_{\omega}']
            self.raw = pd.DataFrame(columns=columns)
            self.time_init = time_now

        # Robot 2D pose (x, y, theta) and velocity
        r_x = robot.pose2D.x
        r_y = robot.pose2D.y
        r_theta = robot.pose2D.theta
        r_v = robot.twist.linear.x
        r_w = robot.twist.angular.z
        r_g = robot.goal
        
        # Human 2D pose (x, y, theta) and velocity            
        h_x = human.pose2D.x
        h_y = human.pose2D.y
        h_theta = human.pose2D.theta
        h_v = human.twist.linear.x
        h_w = human.twist.angular.z
        h_g = human.goal

        self.raw.loc[len(self.raw)] = {'time': robot.header.stamp.to_sec(),
                                     'r_{gx}': r_g.x, 'r_{gy}': r_g.y,
                                     'r_x': r_x, 'r_y': r_y, 
                                     'r_{\theta}': r_theta, 'r_v': r_v, 'r_{\omega}': r_w,
                                     'h_{gx}': h_g.x, 'h_{gy}': h_g.y,
                                     'h_x': h_x, 'h_y': h_y,
                                     'h_{\theta}': h_theta, 'h_v': h_v, 'h_{\omega}': h_w,
                                    }
        
        
    def subsampling(self, df: pd.DataFrame, dt, tol = 0.01):
        """
        subsampling the dataframe taking a sample each dt secs
        
        Args:
            dt (float): subsampling step
            tol (float): tolerance
            
        Returns:
            pd.DataFrame: subsampled dataframe
        """
        sd = pd.DataFrame(columns=df.columns)
        sd.loc[0] = df.loc[0]
        init_t = df.time.values[0]
        for i in range(1, len(df)):
            if df.time.values[i] - init_t >= dt - tol:
                sd.loc[i] = df.loc[i]
                init_t = df.time.values[i]
        return sd.reset_index(drop=True)
        

if __name__ == '__main__':

    # Create data pool directory
    os.makedirs(DATA_DIR, exist_ok=True)
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    dc = DataCollector()

    while not rospy.is_shutdown():
        rate.sleep()