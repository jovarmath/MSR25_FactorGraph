import numpy as np
import subprocess
import os
import time

from srcpython.IMUStochasticParameter import *
from srcpython.iSAMConfig import *

from srcpython.tuningtools.evaluateATR import evaluate_trajectopy_atr

class gridsearch:
    def __init__(self):
        
        # Lists for the gridsearch
        self.sig_preintegration: np.ndarray
        self.sig_acceleration: np.ndarray
        self.sig_gyroscope: np.ndarray
        self.sig_acceleration_bias: np.ndarray
        self.sig_gyroscope_bias: np.ndarray

        # Config files for iSAM
        self.iSAMConfig = iSAMConfig()
        self.IMUparams = IMUStochasticParameter()
    
    def run( self, shell_in, ATRoutpath, reference_tr_path, print_to_termial: bool = False ):

        self.initialize( path = "02_trajectory_estimation/config/SBGsystem/" )

        # Iteration counter
        it_cnt = 0
        numer_of_iterations = len(self.sig_preintegration) * len(self.sig_acceleration) * len(self.sig_gyroscope) * len(self.sig_acceleration_bias) * len(self.sig_gyroscope_bias)

        with open( ATRoutpath + "/ATR.txt", 'w', encoding='utf-8') as file:

            # Write header to result file
            file.write( "Iteration, RMS along [m], RMS h_across [m], RMS v_across [m], RMSroll [deg], RMSpitch [deg], RMSyaw [deg], RMStotal [m], SigIntegration [-], SigAcc [m/s^2], SigGyro [rad/s] \n")

            # Loop over integration sigma
            for i in np.arange(0, len(self.sig_preintegration) ):
                
                # Loop over acceleration sigma
                for j in np.arange(0, len(self.sig_acceleration) ):
                    
                    # Loop over gyroscope sigma
                    for k in np.arange(0, len(self.sig_gyroscope) ):

                        # Loop over acceleration bias sigma
                        for l in np.arange(0, len(self.sig_acceleration_bias) ):
                                
                            # Loop over gyroscope bias sigma
                            for m in np.arange(0, len(self.sig_gyroscope_bias) ):

                                # Set preintegration sigma
                                self.iSAMConfig.set_preintegrationSig( self.sig_preintegration[i] )
                                
                                # Set acceleration and gyroscope sigma
                                self.IMUparams.setsigaccxyz( self.sig_acceleration[j] )
                                self.IMUparams.setsiggyroxyz( self.sig_gyroscope[k] )

                                # Set acceleration and gyroscope bias sigma
                                self.IMUparams.setsigbiasaccxyz( self.sig_acceleration_bias[l] )
                                self.IMUparams.setsigbiasgyroxyz( self.sig_gyroscope_bias[m] )

                                # Write config files to .jsons
                                self.IMUparams.writetojson( path="02_trajectory_estimation/config/SBGsystem/", filename="IMUparams_.json" )
                                self.iSAMConfig.writetojson( path="02_trajectory_estimation/config/SBGsystem/", filename="iSAMConfig_.json" )

                                # Optimize trajectory using isam2
                                #
                                
                                start_time = time.time()
                                print(f"{start_time} [INFO] started grid search iteration: (" + str(it_cnt+1) + " / " + str(numer_of_iterations) + ")\n" )

                                print("SETTINGS: " )
                                print( "[INFO] Integration       sigma: {:.8f}".format(self.sig_preintegration[i][0]) )
                                print( "[INFO] Acceleration      sigma: {:.8f}".format(self.sig_acceleration[j][0]) )
                                print( "[INFO] Gyroscope         sigma: {:.8f}".format(self.sig_gyroscope[k][0]) )
                                print( "[INFO] Acceleration bias sigma: {:.8f}".format(self.sig_acceleration_bias[l][0]) )
                                print( "[INFO] Gyroscope bias    sigma: {:.8f}".format(self.sig_gyroscope_bias[m][0]) + "\n" )
                                
                                print( "OPTIMIZING Trajectory" )

                                # Run C++ isam code
                                process = subprocess.Popen(
                                ['sh', "02_trajectory_estimation/run_isam.sh"],
                                env={**os.environ, **shell_in},
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True
                                )
                                
                                if print_to_termial:
                                    # Print output in real-time
                                    for stdout_line in iter(process.stdout.readline, ''):
                                        print(stdout_line, end='')

                                    for stderr_line in iter(process.stderr.readline, ''):
                                        print(stderr_line, end='')

                                    process.stdout.close()
                                    process.stderr.close()
                                process.wait()

                                # Evaluate error
                                # 

                                ATE, RMStotal = evaluate_trajectopy_atr( path_T_ = shell_in["PathOUT"] + "T_graph.trj", path_T_ref = reference_tr_path )

                                # Write result file
                                #

                                file.write( "{:.0f}".format(it_cnt)
                                                        + ", {:.8f}".format(float(ATE['RMS Along-Track [m]']))
                                                        + ", {:.8f}".format(float(ATE['RMS Horizontal Cross-Track [m]']))
                                                        + ", {:.8f}".format(float(ATE['RMS Vertical Cross-Track [m]']))
                                                        + ", {:.8f}".format(float(ATE['RMS Roll [°]']))
                                                        + ", {:.8f}".format(float(ATE['RMS Pitch [°]']))
                                                        + ", {:.8f}".format(float(ATE['RMS Yaw [°]']))
                                                        + ", {:.8f}".format(RMStotal)
                                                        + ", {:.8f}".format(self.sig_preintegration[i][0])
                                                        + ", {:.8f}".format(self.sig_acceleration[j][0])
                                                        + ", {:.8f}".format(self.sig_gyroscope[k][0])
                                                        + "\n" )
                                
                                execution_time = time.time() - start_time
                                print(f"{time.time()} [INFO] finished evaluation in {execution_time} [s] ")
                                print( "RESULTS       ATRtotal: {:.8f}".format(RMStotal) )

                                it_cnt+=1





    def readlistsfromjson(self, path, filename):

        # Construct the full path to the JSON file
        filepath = path + filename

        logger.info( "Reading .json gridsearch lists: " + path + filename )
        
        # Open and read the JSON file
        with open(filepath, 'r') as json_file:
            data = json.load(json_file)

        # Access specific elements
        sig_preintegration_vec = data['sigmas_integration']
        self.sig_preintegration = [sig_preintegration_vec[i] * np.array([1, 1, 1]) for i in range(len(sig_preintegration_vec))]

        sig_acceleration_vec = data['sigmas_acceleration']
        self.sig_acceleration = [sig_acceleration_vec[i] * np.array([1, 1, 1]) for i in range(len(sig_acceleration_vec))]

        sig_gyroscope_vec = data['sigmas_gyroscope']
        self.sig_gyroscope = [sig_gyroscope_vec[i] * np.array([1, 1, 1]) for i in range(len(sig_gyroscope_vec))]

        sig_acceleration_bias_vec = data['sigmas_acceleration_bias']
        self.sig_acceleration_bias = [sig_acceleration_bias_vec[i] * np.array([1, 1, 1]) for i in range(len(sig_acceleration_bias_vec))]

        sig_gyroscope_bias_vec = data['sigmas_gyroscope_bias']
        self.sig_gyroscope_bias = [sig_gyroscope_bias_vec[i] * np.array([1, 1, 1]) for i in range(len(sig_gyroscope_bias_vec))]

    def initialize(self, path ):
        self.iSAMConfig.readfromjson( path, "iSAMConfig.json" )
        self.IMUparams.readfromjson( path, "IMUparams.json" )
        

