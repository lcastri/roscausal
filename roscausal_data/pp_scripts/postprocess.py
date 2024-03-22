import argparse
import os
import stat
import numpy as np
import pandas as pd
from shapely.geometry import *
import math


def wrap(angle, lower_bound, upper_bound):
    """
    Wrap an angle to be within the specified bounds.

    Args:
        angle (float): The angle to be wrapped.
        lower_bound (float): The lower bound for the angle.
        upper_bound (float): The upper bound for the angle.

    Returns:
        float: The wrapped angle.
    """
    range_width = upper_bound - lower_bound
    wrapped_angle = (angle - lower_bound) % range_width + lower_bound

    return wrapped_angle


class Agent():
    def __init__(self, name, x, y, theta, v, omega) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        
    def p(self, t):
        """
        Position

        Args:
            t (int): time index

        Returns:
            Point: position
        """
        return Point(self.x[t], self.y[t])
    
    def dv(self, t):
        """
        Decomposed velocity

        Args:
            t (int): time index

        Returns:
            Point: decomposed velocity
        """
        return Point(self.v[t]*math.cos(self.theta[t]), self.v[t]*math.sin(self.theta[t]))
    
    def dist(self, t, obs):
        """
        Distance to obstacle

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: distance
        """
        return self.p(t).distance(obs.p(t))
    
    
    def heading(self, t, obj):
        """
        Heading angle
        
        Args:
            t (int): time index
            obj (Agent): object
            
        Returns:
            float: heading angle
        """
        
        angle = wrap(2*np.pi - wrap(math.atan2(obj.p(t).y - self.p(t).y, obj.p(t).x - self.p(t).x) - wrap(self.theta[t], 0, 2*np.pi), 0, 2*np.pi), -np.pi, np.pi)
        return angle
    
    
    def risk(self, t, obs):
        """
        Risk

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: risk value
        """

        
        risk = self.v[t]
        collision = False
        
        # Calculate relative velocity vector
        Vrel = Point(obs.dv(t).x - self.dv(t).x, obs.dv(t).y - self.dv(t).y)
            
        # Compute the slope of the line AB and line PAB ⊥ AB
        slope_AB = (obs.p(t).y - self.p(t).y) / (obs.p(t).x - self.p(t).x)  # Rise over run
        slope_PAB = -1 / slope_AB
        # Choose distances to the left and right of point B

        # Calculate coordinates for two points along the perpendicular line
        # Calculate the change in x and y based on the fixed horizontal distance
        delta_x = OBS_SIZE / (1 + slope_PAB ** 2) ** 0.5
        delta_y = delta_x * slope_PAB
        # Calculate coordinates for two points along the perpendicular line
        left = Point(obs.p(t).x - delta_x, obs.p(t).y - delta_y)
        right = Point(obs.p(t).x + delta_x, obs.p(t).y + delta_y)
        # Cone
        cone_origin = Point(self.p(t).x, self.p(t).y)               
        cone = Polygon([cone_origin, left, right])
        
        P = Point(cone_origin.x + self.dv(t).x, cone_origin.y + self.dv(t).y)
        collision = P.within(cone) and self.p(t).distance(obs.p(t)) < SAFE_DIST
        if collision:
            time_collision_measure = self.p(t).distance(obs.p(t)) / math.sqrt(Vrel.x**2 + Vrel.y**2)
            steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
            risk = risk + 1/time_collision_measure + steering_effort_measure
                    
        return math.exp(risk)
           

if __name__ == '__main__': 
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", help="CSV file name")
    parser.add_argument("--data_dir", help="raw data dir")
    parser.add_argument("--pp_data_dir", help="postprocess data dir")
    parser.add_argument("--obs_size", help="Obstacle increase size (radius) for risk calculation")
    parser.add_argument("--safe_dist", help="Safety distance for risk calculation")
    parser.add_argument("--sel_agent", help="Selected agent")
    args = parser.parse_args()

    CSV = args.csv
    DATA_DIR = args.data_dir
    PP_DATA_DIR = args.pp_data_dir
    OBS_SIZE = float(args.obs_size)
    SAFE_DIST = float(args.safe_dist)
    
    INPUT_CSV = DATA_DIR + '/' + CSV
    OUTPUT_CSV = PP_DATA_DIR + '/' + CSV

    # Read the CSV into a pandas DataFrame
    data = pd.read_csv(INPUT_CSV)
    
    R = Agent("R", data["r_x"], data["r_y"], data["r_{\theta}"], data["r_v"], data["r_{\omega}"])
    H = Agent("H", data["h_x"], data["h_y"], data["h_{\theta}"], data["h_v"], data["h_{\omega}"])
    RG = Agent("RG", data["r_{gx}"], data["r_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]))
    HG = Agent("HG", data["h_{gx}"], data["h_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]))
            
    df = pd.DataFrame(columns=["h_v", r"h_{\theta}", r"h_{\theta_{g}}", "h_{d_g}", "h_{risk}", r"h_{\omega}", r"h_{d_{obs}}"])  
    
    for i in range(1, len(data)):
                
        df.loc[i] = {"h_v" : H.v[i],
                     r"h_{\theta}" : H.theta[i], 
                     r"h_{\theta_{g}}" : H.heading(i, HG), 
                     "h_{d_g}" : H.dist(i, HG), 
                     "h_{risk}" : H.risk(i-1, R), 
                     r"h_{\omega}" : H.omega[i],
                     r"h_{d_{obs}}" : H.dist(i, R),
                     }

    # Save the processed data to another CSV file
    df = df[1:]
    df.to_csv(OUTPUT_CSV, index=False)
    
    os.chmod(INPUT_CSV, stat.S_IWUSR | stat.S_IRUSR | stat.S_IXUSR)
    os.remove(INPUT_CSV)