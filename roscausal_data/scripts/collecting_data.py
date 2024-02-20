#!/usr/bin/env python

from datetime import datetime
import math
import subprocess
import rospy
import pandas as pd
import os
import message_filters
from roscausal_msgs.msg import HumanState, Humans, RobotState


NODE_NAME = "roscausal_data"
NODE_RATE = 10 # [Hz]

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
        sub_people = message_filters.Subscriber('/roscausal/human', Humans)
                        
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_robot, 
                                                                sub_people], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.cb_handle_data)
                    

                              
    def cb_handle_data(self, robot: RobotState, people: Humans):
        """
        Synchronized callback

        Args:
            robot (RobotState): robot state
            people (Humans): tracked person
        """
        time_now = robot.header.stamp.to_sec()
        if self.time_init is None or (time_now - self.time_init >= TS_LENGTH):
            
            # Save currect dataframe
            if self.raw is not None:
                # this is to make sure that the dataframe contains data with time in ascending order.
                # with ApproximateTimeSynchronizer might not be always true
                self.raw.sort_values(by = 'time', ascending = True, inplace = True, ignore_index = True)
                
                # subsample your dataset
                if SUBSAMPLING: self.raw = self.subsampling(self.raw, DT)
                timestamp_str = datetime.now().strftime(ID_FORMAT)
                csv_name = CSV_PREFIX + timestamp_str + '.csv'
                rospy.logwarn("CSV file saved: " + csv_name)
                self.raw.to_csv(DATA_DIR + '/' + csv_name, index=False)
                
                if PP_SCRIPT != "":
                    rospy.logwarn("Postprocessing file: " + csv_name)
                    subprocess.check_call(["python", PP_SCRIPT_DIR + PP_SCRIPT, "--csv", csv_name,
                                                                                "--data_dir", DATA_DIR,
                                                                                "--pp_data_dir", PP_DATA_DIR,
                                                                                "--obs_size", OBS_SIZE,
                                                                                "--safe_dist", SAFE_DIST,
                                                                                "--sel_agent", SEL_AGENT])
            
            # Init dataframe
            columns = ['time', 'r_{gx}', 'r_{gy}', 'r_x', 'r_y', 'r_{\theta}', 'r_v', 'r_{\omega}']
            self.raw = pd.DataFrame(columns=columns)
            self.time_init = time_now

        new_row_loc = len(self.raw)

        r_x = robot.pose2D.x
        r_y = robot.pose2D.y
        r_theta = robot.pose2D.theta
        r_v = math.sqrt(robot.twist.linear.x**2 + robot.twist.linear.y**2)
        r_w = robot.twist.angular.z
        r_g = robot.goal
        
        self.raw.loc[new_row_loc] = {'time': robot.header.stamp.to_sec(),
                                    'r_{gx}': r_g.x, 'r_{gy}': r_g.y,
                                    'r_x': r_x, 'r_y': r_y, 
                                    'r_{\theta}': r_theta, 'r_v': r_v, 'r_{\omega}': r_w,}
        
        for human in people.humans:        
            h_id = human.id
            if f'h_{h_id}_x' not in self.raw.columns: self.raw[f'h_{h_id}_x'] = None    
            if f'h_{h_id}_y' not in self.raw.columns: self.raw[f'h_{h_id}_y'] = None    
            if f'h_{h_id}'+'_{\theta}' not in self.raw.columns: self.raw[f'h_{h_id}'+'_{\theta}'] = None    
            if f'h_{h_id}_v' not in self.raw.columns: self.raw[f'h_{h_id}_v'] = None    
            if f'h_{h_id}'+'_{\omega}' not in self.raw.columns: self.raw[f'h_{h_id}'+'_{\omega}'] = None    
            if f'h_{h_id}'+'_{gx}' not in self.raw.columns: self.raw[f'h_{h_id}'+'_{gx}'] = None    
            if f'h_{h_id}'+'_{gy}' not in self.raw.columns: self.raw[f'h_{h_id}'+'_{gy}'] = None    
            self.raw.loc[new_row_loc,f'h_{h_id}_x'] = human.pose2D.x
            self.raw.loc[new_row_loc,f'h_{h_id}_y'] = human.pose2D.y
            self.raw.loc[new_row_loc,f'h_{h_id}'+'_{\theta}'] = human.pose2D.theta
            self.raw.loc[new_row_loc,f'h_{h_id}_v'] = math.sqrt(human.twist.linear.x**2 + human.twist.linear.y**2)
            self.raw.loc[new_row_loc,f'h_{h_id}'+'_{\omega}'] = human.twist.angular.z
            self.raw.loc[new_row_loc,f'h_{h_id}'+'_{gx}'] = human.goal.x
            self.raw.loc[new_row_loc,f'h_{h_id}'+'_{gy}'] = human.goal.y

        
        
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

    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    TS_LENGTH = float(rospy.get_param("~ts_length", default = 150)) # [s]
    DATA_DIR = str(rospy.get_param("~data_dir"))
    DT = float(rospy.get_param("~dt", default = 0.1))
    SUBSAMPLING = bool(rospy.get_param("~subsampling", default = False))
    ID_FORMAT = str(rospy.get_param("~id_format", default = '%Y%m%d_%H%M%S'))
    CSV_PREFIX = str(rospy.get_param("~csv_prefix", default = 'data_'))
    PP_DATA_DIR = str(rospy.get_param("~pp_data_dir"))
    PP_SCRIPT_DIR = str(rospy.get_param("~pp_script_dir"))
    PP_SCRIPT = str(rospy.get_param("~pp_script"))
    OBS_SIZE = str(rospy.get_param("~obs_size"))
    SAFE_DIST = str(rospy.get_param("~safe_dist"))
    SEL_AGENT = str(rospy.get_param("~sel_agent"))
    
    # Create data pool directory
    os.makedirs(DATA_DIR, exist_ok=True)
    if PP_SCRIPT != "": os.makedirs(PP_DATA_DIR, exist_ok=True)
    
    dc = DataCollector()

    while not rospy.is_shutdown():
        rate.sleep()