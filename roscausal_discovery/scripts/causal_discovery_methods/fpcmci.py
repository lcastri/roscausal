from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType, ImageExt

import rospy


def run(csvpath, csvname, alpha, minlag, maxlag, resdir):

    df = Data(csvpath)
        
    cdm = FPCMCI(df, 
                 f_alpha = alpha,
                 pcmci_alpha = alpha,
                 min_lag = minlag, 
                 max_lag = maxlag, 
                 sel_method = TE(TEestimator.Gaussian), 
                 val_condtest = GPDC(significance = 'analytic', gp_params = None),
                 verbosity = CPLevel.NONE,
                 neglect_only_autodep = True,
                 clean_cls=False,
                 resfolder = resdir + '/' + csvname if resdir != "" else None)
        
    feature, cm = cdm.run()
    
    cs = None
    val = None
    pval = None
    if resdir != "":
        if feature is not None and len(feature) > 0:   
            cdm.dag(label_type = LabelType.Lag, max_width=3, node_size=5, font_size=16)
            cdm.timeseries_dag(max_width=3, node_size=5, font_size=16, node_proximity=4)
            cs = cm.get_skeleton()
            val = cm.get_val_matrix()
            pval = cm.get_pval_matrix()
    

    return feature, cs, val, pval, cdm.dag_path + ImageExt.PNG.value, cdm.ts_dag_path + ImageExt.PNG.value