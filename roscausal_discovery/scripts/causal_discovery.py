#!/usr/bin/env python

from datetime import datetime
from enum import Enum
import glob
import os
import subprocess
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType
import rospy
import pandas as pd
from roscausal_msgs.msg import CausalModel
from std_msgs.msg import Header
from sklearn.utils._testing import ignore_warnings
from sklearn.exceptions import ConvergenceWarning
import stat


class CausalDiscoveryMethod(Enum):
    PCMCI = "pcmci"
    FPCMCI = "fpcmci"


NODE_NAME = "roscausal_discovery"
NODE_RATE = 10 # [Hz]
CDM = str(rospy.get_param("~cd_method", default = "fpcmci"))
FALPHA = float(rospy.get_param("~filter_alpha", default = 0.05))
ALPHA = float(rospy.get_param("~sig_alpha", default = 0.05))
MINLAG = int(rospy.get_param("~min_lag", default = 1))
MAXLAG = int(rospy.get_param("~max_lag", default = 1))
DATA_DIR = str(rospy.get_param("~data_dir", default = '/root/shared/')) + 'data_pool'
RES_DIR = str(rospy.get_param("~res_dir", default = '/root/shared/')) + 'cm_pool'
POSTPROCESS_DATA_DIR = str(rospy.get_param("~postprocess_data_dir", default = '/root/shared/')) + 'postprocess_pool'
ID_FORMAT = str(rospy.get_param("~id_format", default = '%Y%m%d_%H%M%S'))
CSV_PREFIX = str(rospy.get_param("~cas_prefix", default = 'data_'))
POSTPROCESS_SCRIPT_DIR = str(rospy.get_param("~postprocess_script_dir", ""))
POSTPROCESS_SCRIPT = str(rospy.get_param("~postprocess_script", ""))
VARS = ["r" + v for v in str(rospy.get_param("~vars")).split(",")]


class CausalDiscovery():
    
    def __init__(self, df: pd.DataFrame, dfname) -> None:
        """
        CausalDiscovery constructor

        Args:
            df (pd.DataFrame): csv file converted into a dataframe
            dfname (str): csv file name
        """
        self.df = df
        self.dfname = dfname
        
        
    @ignore_warnings(category=ConvergenceWarning)
    def run(self):
        """
        Run causal discovery algorithm

        Returns:
            list(str): list of selected features
            DAG: causal model
        """
        df = Data(self.df)
        df.shrink(VARS)
        
        cdm = FPCMCI(df, 
                     f_alpha = FALPHA,
                     pcmci_alpha = ALPHA,
                     min_lag = MINLAG, 
                     max_lag = MAXLAG, 
                     sel_method = TE(TEestimator.Gaussian), 
                     val_condtest = GPDC(significance = 'analytic', gp_params = None),
                     verbosity = CPLevel.NONE,
                     neglect_only_autodep = True,
                     resfolder = RES_DIR + '/' + self.dfname)
        
        if CDM == CausalDiscoveryMethod.FPCMCI.value:
            feature, causalmodel = cdm.run()
        elif CDM == CausalDiscoveryMethod.PCMCI.value:
            feature, causalmodel = cdm.run_pcmci()
        
        if len(feature) > 0:   
            cdm.dag(label_type = LabelType.NoLabels)
            cdm.timeseries_dag()
        
        return feature, causalmodel
        

def extract_timestamp_from_filename(file_path, file_prefix=CSV_PREFIX, file_extension='.csv'):
    """
    Extract timestamp from the file_path

    Args:
        file_path (str): file path
        file_prefix (str, optional): csv file prefix. Defaults to CSV_PREFIX.
        file_extension (str, optional): file extenstion. Defaults to '.csv'.

    Returns:
        str: timestamp
    """
    # Extract the timestamp from the file name
    file = os.path.basename(file_path)
    start_index = len(file_prefix)
    end_index = file.find(file_extension)
    timestamp_str = file[start_index:end_index]

    # Convert the timestamp string to a datetime object
    return datetime.strptime(timestamp_str, ID_FORMAT)


def get_file(file_prefix=CSV_PREFIX, file_extension='.csv'):
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
    file_pattern = os.path.join(DATA_DIR, f'{file_prefix}*{file_extension}')

    # List files in the directory matching the pattern
    files = glob.glob(file_pattern)

    if not files:
        return None, None  # No files found

    # Get the oldest file based on the extracted timestamp
    oldest_file = min(files, key=lambda file: extract_timestamp_from_filename(file))
    filename = os.path.basename(oldest_file)
    return oldest_file, filename[:filename.find(file_extension)]
                
                
if __name__ == '__main__':
    # Create res pool directory
    os.makedirs(RES_DIR, exist_ok=True)
    if POSTPROCESS_SCRIPT != "": os.makedirs(POSTPROCESS_DATA_DIR, exist_ok=True)
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)

    # Publisher
    pub_causal_model = rospy.Publisher('/hri/causal_model', CausalModel, queue_size=10)
    
    rospy.logwarn("Waiting for a csv file...")
    while not rospy.is_shutdown():
        
        csv, name = get_file()
        if csv is not None:
            if POSTPROCESS_SCRIPT != "":
                rospy.logwarn("Postprocessing file: " + name)
                subprocess.check_call(["python", POSTPROCESS_SCRIPT_DIR + POSTPROCESS_SCRIPT, "--csv", name])
                pp_csv = '/'.join([POSTPROCESS_DATA_DIR, os.path.basename(csv)])
            
            if POSTPROCESS_SCRIPT != "":
                rospy.logwarn("Causal analysis on: " + pp_csv)
                d = pd.read_csv(pp_csv)
            else:
                rospy.logwarn("Causal analysis on: " + csv)
                d = pd.read_csv(csv)
                
            dc = CausalDiscovery(d, name)
            f, cm = dc.run()
            
            if len(f) > 0:
                msg = CausalModel()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                
                cs = cm.get_skeleton()
                val = cm.get_val_matrix()
                pval = cm.get_pval_matrix()
                rospy.logwarn("CAUSAL STRUCTURE: " + str(cs))
                rospy.logwarn("VAL MATRIX: " + str(val))
                rospy.logwarn("PVAL MATRIX: " + str(pval))
                msg.features = f
                msg.causal_structure.data = cs.flatten().tolist()
                msg.val_matrix.data = val.flatten().tolist()
                msg.pval_matrix.data = pval.flatten().tolist()
                msg.original_shape = list(cs.shape)
                pub_causal_model.publish(msg)
                
            rospy.logwarn("Removing file: " + csv)
            os.chmod(csv, stat.S_IWUSR | stat.S_IRUSR | stat.S_IXUSR)
            os.remove(csv)
            if POSTPROCESS_SCRIPT != "":
                rospy.logwarn("Removing file: " + pp_csv)
                os.chmod(pp_csv, stat.S_IWUSR | stat.S_IRUSR | stat.S_IXUSR)
                os.remove(pp_csv)
                
        rate.sleep()