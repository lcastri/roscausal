import argparse
import numpy as np
import pandas as pd
from shapely.geometry import *
import math
import rospy

DATA_DIR = str(rospy.get_param("/mytiago_causal_discovery/data_dir", default = '/root/shared/')) + 'data_pool'
POSTPROCESS_DIR = str(rospy.get_param("/mytiago_causal_discovery/postprocess_dir", default = '/root/shared/')) + 'postprocess_pool'

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
        decomposed velocity

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
    
    def bearing(self, t, obs):
        """
        Bearing angle

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: bearing angle
        """
        return math.atan2(obs.p(t).y - self.p(t).y, obs.p(t).x - self.p(t).x)

    def risk(self, t, obs):
        """
        risk of collision

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: risk
        """
        
        risk = self.v[t]
        
        # Calculate relative velocity vector
        Vrel = Point(obs.dv(t).x - self.dv(t).x, obs.dv(t).y - self.dv(t).y)
               
        # Cone origin translated by Vb
        cone_origin = Point(self.p(t).x + obs.dv(t).x, self.p(t).y + obs.dv(t).y)
                            
        # Straight line from A to B = r_{a_b}
        AB = LineString([self.p(t), obs.p(t)])
            
        # PAB ⊥ AB passing through B
        left = AB.parallel_offset(5, 'left')
        right = AB.parallel_offset(5, 'right')

        c = left.boundary.geoms[1]
        d = right.boundary.geoms[0]
        PAB = LineString([c, d])
                    
        # Straight line ⊥ to r_{a_b} and passing through b
        B_encumbrance = obs.p(t).buffer(1.5)
        if PAB.intersects(B_encumbrance): 
            inter = PAB.intersection(B_encumbrance).xy
            inter_l = Point(inter[0][0] + obs.dv(t).x, inter[1][0] + obs.dv(t).y)
            inter_r = Point(inter[0][1] + obs.dv(t).x, inter[1][1] + obs.dv(t).y)
                    
            # Cone
            cone = Polygon([cone_origin, inter_l, inter_r])
            P = Point(cone_origin.x + Vrel.x, cone_origin.y + Vrel.y)
            if P.within(cone):
                time_collision_measure = self.dist(t, obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
                steering_effort_measure = min(P.distance(LineString([cone_origin, inter_l])), P.distance(LineString([cone_origin, inter_r])))           
                risk = risk + 1/time_collision_measure + steering_effort_measure
                    
        return math.exp(risk)

       

if __name__ == '__main__': 
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", help="CSV file name")
    args = parser.parse_args()
    csv_name = args.csv
    
    # DATA_DIR = '/home/lucacastri/git/TIAGo-docker/shared/data_pool'
    # POSTPROCESS_DIR = '/home/lucacastri/git/'
    # csv_name = 'data_20240124_145254'
    INPUT_CSV = DATA_DIR + '/' + csv_name + '.csv'
    OUTPUT_CSV = POSTPROCESS_DIR + '/' + csv_name + '.csv'

    # Read the CSV into a pandas DataFrame
    data = pd.read_csv(INPUT_CSV)
    
    R = Agent("R", data["r_x"], data["r_y"], data["r_{\theta}"], data["r_v"], data["r_{\omega}"])
    H = Agent("H", data["h_x"], data["h_y"], data["h_{\theta}"], data["h_v"], data["h_{\omega}"])
    RG = Agent("RG", data["r_{gx}"], data["r_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]))
    HG = Agent("HG", data["h_{gx}"], data["h_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]))
            
    df = pd.DataFrame(columns=["r_v", r"r_{\theta}", r"r_{\theta_{g}}", "r_{d_g}", "r_{risk}", r"r_{\omega}", r"r_{d_{obs}}", 
                               "h_v", r"h_{\theta}", r"h_{\theta_{g}}", "h_{d_g}", "h_{risk}", r"h_{\omega}", r"h_{d_{obs}}"])
            
            
    df.loc[0] = {"r_v" : R.v[0], 
                 "r_{\theta}" : R.theta[0], 
                 "r_{\theta_{g}}" : R.bearing(0, RG), 
                 "r_{d_g}" : R.dist(0, RG), 
                 "r_{risk}" : R.risk(0, H), 
                 "r_{\omega}" : R.omega[0],
                 "r_{d_{obs}}" : R.dist(0, H), 
                 "h_v" : H.v[0], 
                 "h_{\theta}" : H.theta[0], 
                 "h_{\theta_{g}}" : H.bearing(0, HG), 
                 "h_{d_g}" : H.dist(0, HG), 
                 "h_{risk}" : H.risk(0, R), 
                 "h_{\omega}" : H.omega[0],
                 "h_{d_{obs}}" : H.dist(0, R),
                 }
            
    for i in range(1, len(data)):
                
        df.loc[i] = {"r_v" : R.v[i], 
                     r"r_{\theta}" : R.theta[i], 
                     r"r_{\theta_{g}}" : R.bearing(i, RG), 
                     "r_{d_g}" : R.dist(i, RG), 
                     "r_{risk}" : R.risk(i-1, H), 
                     r"r_{\omega}" : R.omega[i],
                     r"r_{d_{obs}}" : R.dist(i, H), 
                     "h_v" : H.v[i],
                     r"h_{\theta}" : H.theta[i], 
                     r"h_{\theta_{g}}" : H.bearing(i, HG), 
                     "h_{d_g}" : H.dist(i, HG), 
                     "h_{risk}" : H.risk(i-1, R), 
                     r"h_{\omega}" : H.omega[i],
                     r"h_{d_{obs}}" : H.dist(i, R),
                     }

    # Save the processed data to another CSV file
    df = df[1:]
    df.to_csv(OUTPUT_CSV, index=False)