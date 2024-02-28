#!/usr/bin/env python

from datetime import datetime
import glob
import os
import rospy
from roscausal_msgs.msg import CausalModel
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import stat
import importlib
from inspect import isfunction
from fpcmci.basics.constants import LabelType
from cv_bridge import CvBridge
import cv2

NODE_NAME = "roscausal_discovery"
NODE_RATE = 10 # [Hz]


class CausalDiscovery():
    
    def __init__(self, csv_path, dfname) -> None:
        """
        CausalDiscovery constructor

        Args:
            df (pd.DataFrame): csv file converted into a dataframe
            dfname (str): csv file name
        """
        self.csv_path = csv_path
        self.dfname = dfname
        

    def run(self):
        module = importlib.import_module("causal_discovery_methods." + CDM)
        for attribute_name in dir(module):
            attribute = getattr(module, attribute_name)
            if isfunction(attribute) and attribute_name == "run":
                features, cs, val, pval, dagpath, tsdagpath = attribute(self.csv_path, self.dfname, ALPHA, MINLAG, MAXLAG, RES_DIR)
        return features, cs, val, pval, dagpath, tsdagpath
        

def extract_timestamp_from_filename(file_path, file_extension='.csv'):
    """
    Extract timestamp from the file_path

    Args:
        file_path (str): file path
        file_extension (str, optional): file extenstion. Defaults to '.csv'.

    Returns:
        str: timestamp
    """
    # Extract the timestamp from the file name
    file = os.path.basename(file_path)
    start_index = len(CSV_PREFIX)
    end_index = file.find(file_extension)
    timestamp_str = file[start_index:end_index]

    # Convert the timestamp string to a datetime object
    return datetime.strptime(timestamp_str, ID_FORMAT)


def get_file(file_extension='.csv'):
    """
    Get file to process

    Args:
        file_prefix (str, optional): csv file prefix. Defaults to CSV_PREFIX.
        file_extension (str, optional): file extenstion. Defaults to '.csv'.

    Returns:
        str: file path
        str: file name
    """
    # Construct the file pattern based on the prefix and extension
    file_pattern = os.path.join(DATA_DIR, f'{CSV_PREFIX}*{file_extension}')

    # List files in the directory matching the pattern
    files = glob.glob(file_pattern)

    if not files:
        return None, None  # No files found

    # Get the oldest file based on the extracted timestamp
    oldest_file = min(files, key=lambda file: extract_timestamp_from_filename(file))
    filename = os.path.basename(oldest_file)
    return oldest_file, filename[:filename.find(file_extension)]

               
                
if __name__ == '__main__':
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    CDM = str(rospy.get_param("~cd_method", default = "fpcmci"))
    ALPHA = float(rospy.get_param("~sig_alpha", default = 0.05))
    MINLAG = int(rospy.get_param("~min_lag", default = 1))
    MAXLAG = int(rospy.get_param("~max_lag", default = 1))
    DATA_DIR = str(rospy.get_param("~data_dir", default = '/root/shared/'))
    RES_DIR = str(rospy.get_param("~res_dir", default = '/root/shared/'))
    ID_FORMAT = str(rospy.get_param("~id_format", default = '%Y%m%d_%H%M%S'))
    CSV_PREFIX = str(rospy.get_param("~css_prefix", default = 'data_'))
    
    # Create res pool directory
    if RES_DIR != "": os.makedirs(RES_DIR, exist_ok=True)   

    # Publisher
    pub_causal_model = rospy.Publisher('/roscausal/causal_model', CausalModel, queue_size=10)
    pub_dag = rospy.Publisher('/roscausal/dag', Image, queue_size=10)
    pub_tsdag = rospy.Publisher('/roscausal/tsdag', Image, queue_size=10)
    
    rospy.logwarn("Waiting for a csv file...")
    while not rospy.is_shutdown():
        
        csv, name = get_file()
        if csv is not None:
            rospy.logwarn("Causal analysis on: " + csv)
                
            dc = CausalDiscovery(csv, name)
            f, cs, val, pval, dagpath, tsdagpath = dc.run()
            
            if len(f) > 0:
                
                # Publish CausalModel
                msg = CausalModel()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                
                rospy.logwarn("Features: " + str(f))
                rospy.logwarn("Causal Structure: " + str(cs))
                rospy.logwarn("Val Matrix: " + str(val))
                rospy.logwarn("Pval Matrix: " + str(pval))
                msg.features = f
                msg.causal_structure.data = cs.flatten().tolist()
                msg.val_matrix.data = val.flatten().tolist()
                msg.pval_matrix.data = pval.flatten().tolist()
                msg.original_shape = list(cs.shape)
                pub_causal_model.publish(msg)
                
                bridge = CvBridge()
                
                # Publish DAG
                img = cv2.imread(dagpath)
                img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                pub_dag.publish(img_msg)
                
                # Publish tsDAG
                img = cv2.imread(tsdagpath)
                img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                pub_tsdag.publish(img_msg)
                
            rospy.logwarn("Removing file: " + csv)
            os.chmod(csv, stat.S_IWUSR | stat.S_IRUSR | stat.S_IXUSR)
            os.remove(csv)
                
        rate.sleep()