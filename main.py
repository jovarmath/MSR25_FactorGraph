import logging
import coloredlogs
import subprocess
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import math
from datetime import datetime
import time

def main():
    # ########################################################################
    # Dataset and system information
    #

    #sys_path = "phenobot-sbg-data" # Datasets: "/23_11_15/03/" "/mnt/d/trajectory-estimation-data/phenobot-sbg-data/23_11_15/03/"
    #sys_path = "SBG-system-data" # Datasets: "/24_07_22/04/"

    # "phenobot-sbg-data":
    # -/mnt/d/trajectory-estimation-data/phenobot-sbg-data/
    # -/mnt/d/trajectory-estimation-data/sbg-data/:

    # dataset = "/23_11_15/03/"

    #dataset = "/24_07_22/04/"

    # Datasets felix:

    # # Input arguments for iSAM
    # SETTINGS = {
    #         'PathDATASET': "/mnt/d/trajectory-estimation-data/" + sys_path + dataset,
    #         'PathCONFIG': "/mnt/d/trajectory-estimation-data/" + sys_path + dataset + "config/",
    #         'PathOUT': "/mnt/d/trajectory-estimation-data/" + sys_path + dataset + "02_trajectory/"    
    # }

    # Datasets manuel:

    # # Input arguments for iSAM
    # SETTINGS = {
    #         'PathDATASET': "../DATA/" + sys_path + dataset,
    #         'PathCONFIG': "../DATA/" + sys_path + dataset + "config/",
    #         'PathOUT': "../DATA/" + sys_path + dataset + "02_trajectory/"    
    # }


    # Example
    # python3 main.py "PhenobotSBG" "/mnt/d/trajectory-estimation-data/" "/23_11_15/03/"
    #

    # ############################################################################################
    # PHENOBOT

    if sys.argv[1] == "PhenobotSBG":
        sys_path = "phenobot-sbg-data"

        #data_path = sys.argv[2]
        #dataset = sys.argv[3]

        SETTINGS = {
                'PathDATASET': sys.argv[2],
                'PathCONFIG':  sys.argv[2] + "config/",
                'PathOUT':     sys.argv[2] + "02_trajectory/"    
        }

    # ############################################################################################
    # SBG

    elif sys.argv[1] == "SBGsystem":
        sys_path = "SBG-system-data"

        data_path = sys.argv[2]
        dataset = sys.argv[3]

        # Input arguments for iSAM
        SETTINGS = {
                'PathDATASET': data_path + sys_path + dataset,
                'PathCONFIG': data_path + sys_path + dataset + "config/",
                'PathOUT': data_path + sys_path + dataset + "02_trajectory/"    
        }

    # ########################################################################
    # Preprocess sensor data
    
    if sys_path == "phenobot-sbg-data":
        subprocess.call("python3 01_preprocessing/SBG_phenobot.py " + SETTINGS['PathDATASET'] + " " + str(int(1)), shell=True)
    if sys_path == "SBG-system-data":
        subprocess.call("python3 01_preprocessing/SBG.py " + SETTINGS['PathDATASET'] + " " + str(int(1)), shell=True)
        
    start_time = time.time()

    process = subprocess.Popen( ['sh', "02_trajectory_estimation/run_isam.sh"],
                                env={**os.environ, **SETTINGS},
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True )
    
    # Print output in real-time
    for stdout_line in iter(process.stdout.readline, ''):
        print(stdout_line, end='')

    for stderr_line in iter(process.stderr.readline, ''):
        print(stderr_line, end='')

    process.stdout.close()
    process.stderr.close()
    process.wait()

    execution_time = time.time() - start_time
    print(f"Optimization time factor graph: {execution_time} seconds")


    subprocess.call("python3 03_trajectory_viz/main.py " + SETTINGS['PathDATASET'] + " " + str(int(1)), shell=True)

if __name__ == "__main__":
    main()