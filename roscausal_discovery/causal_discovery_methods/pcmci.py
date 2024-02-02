import argparse
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--csvpath", help="CSV path")
    parser.add_argument("--csvname", help="CSV file name")
    parser.add_argument("--falpha", help="filter significance level")
    parser.add_argument("--alpha", help="causal significance level")
    parser.add_argument("--minlag", help="Minimum time lag")
    parser.add_argument("--maxlag", help="Maximum time lag")
    parser.add_argument("--resdir", help="Result directory")
    args = parser.parse_args()
    
    CSV = args.csvpath
    CSVNAME = args.csvname
    FALPHA = float(args.falpha)
    ALPHA = float(args.alpha)
    MINLAG = int(args.minlag)
    MAXLAG = int(args.maxlag)
    RES_DIR = args.resdir

    df = Data(CSV)
        
    cdm = FPCMCI(df, 
                 f_alpha = FALPHA,
                 pcmci_alpha = ALPHA,
                 min_lag = MINLAG, 
                 max_lag = MAXLAG, 
                 sel_method = TE(TEestimator.Gaussian), 
                 val_condtest = GPDC(significance = 'analytic', gp_params = None),
                 verbosity = CPLevel.NONE,
                 neglect_only_autodep = True,
                 resfolder = RES_DIR + '/' + CSVNAME if RES_DIR != "" else None)
        
    feature, cm = cdm.run_pcmci()
        
    if RES_DIR != "" and len(feature) > 0:   
        cdm.dag(label_type = LabelType.Lag)
        cdm.timeseries_dag()
        cs = cm.get_skeleton()
        val = cm.get_val_matrix()
        pval = cm.get_pval_matrix()
        
        # Return the result as a JSON-formatted string
        result = {
            "Features": feature,
            "Skeleton": cs,
            "ValMatrix": val,
            "PValMatrix": pval,
        }
    else:
        result = {
            "Features": None,
            "Skeleton": None,
            "ValMatrix": None,
            "PValMatrix": None,
        }
        

    print(str(result))