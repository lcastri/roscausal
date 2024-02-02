import argparse
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType


def run(csvpath, csvname, alpha, minlag, maxlag, resdir):
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--csvpath", help="CSV path")
    # parser.add_argument("--csvname", help="CSV file name")
    # parser.add_argument("--falpha", help="filter significance level")
    # parser.add_argument("--alpha", help="causal significance level")
    # parser.add_argument("--minlag", help="Minimum time lag")
    # parser.add_argument("--maxlag", help="Maximum time lag")
    # parser.add_argument("--resdir", help="Result directory")
    # args = parser.parse_args()
    
    # CSV = args.csvpath
    # CSVNAME = args.csvname
    # FALPHA = float(args.falpha)
    # ALPHA = float(args.alpha)
    # MINLAG = int(args.minlag)
    # MAXLAG = int(args.maxlag)
    # RES_DIR = args.resdir

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
                 resfolder = resdir + '/' + csvname if resdir != "" else None)
        
    feature, cm = cdm.run()
    
    cs = None
    val = None
    pval = None
    if resdir != "" and len(feature) > 0:   
        cdm.dag(label_type = LabelType.Lag)
        cdm.timeseries_dag()
        cs = cm.get_skeleton()
        val = cm.get_val_matrix()
        pval = cm.get_pval_matrix()
    

    return feature, cs, val, pval