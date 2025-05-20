import numpy as np
import subprocess
import os
import time

from srcpython.IMUStochasticParameter import *
from srcpython.iSAMConfig import *

from srcpython.tuningtools.evaluateATR import evaluate_trajectopy_atr

class randomsearch:
    def __init__(self):
        
        # Lists for the gridsearch
        self.sig_int_mean: 0
        self.sig_int_sig: 0

        self.sig_acceleration_mean: 0
        self.sig_acceleration_sig: 0

        self.sig_gyroscope_mean: 0
        self.sig_gyroscope_sig: 0

        self.sig_acceleration_bias_mean: 0
        self.sig_acceleration_bias_sig: 0

        self.sig_gyroscope_bias_mean: 0
        self.sig_gyroscope_bias_sig: 0

        # Numer of samples for each parameter
        self.number_samplings = 0

        # Config files for iSAM
        self.iSAMConfig = iSAMConfig()
        self.IMUparams = IMUStochasticParameter()
    
    def run( self, shell_in, ATRoutpath, reference_tr_path, print_to_termial: bool = False ):

        self.initialize( path = "02_trajectory_estimation/config/SBGsystem/" )

        # Iteration counter
        it_cnt = 0

        with open( ATRoutpath + "/ATR.txt", 'w', encoding='utf-8') as file:

            # Write header to result file
            file.write( "Iteration, RMS along [m], RMS h_across [m], RMS v_across [m], RMSroll [deg], RMSpitch [deg], RMSyaw [deg], RMStotal [m], SigIntegration [-], SigAcc [m/s^2], SigGyro [rad/s], SigBiasAcc [m/s^2], SigBiasGyro [rad/s] \n")

            # Loop over integration sigma
            for i in np.arange(0, self.number_samplings ):
            
                # ############################################################
                # Random generate samples from distributions given

                # 1) Integration sigma
                sig_int_i = np.full((1, 3), np.random.normal(self.sig_int_mean, self.sig_int_sig)).flatten()
                self.iSAMConfig.set_preintegrationSig( sig_int_i )

                # 2) Acceleration
                sig_acc_i = np.full((1, 3), np.random.normal(self.sig_acceleration_mean, self.sig_acceleration_sig)).flatten()
                self.IMUparams.setsigaccxyz( sig_acc_i )

                # 3) Gyroscope
                sig_gyro_i = np.full((1, 3), np.random.normal(self.sig_gyroscope_mean, self.sig_gyroscope_sig)).flatten()
                self.IMUparams.setsiggyroxyz( sig_gyro_i )

                # 4) Acceleration bias
                sig_acc_bias_i = np.full((1, 3), np.random.normal(self.sig_acceleration_bias_mean, self.sig_acceleration_bias_sig)).flatten()
                self.IMUparams.setsigbiasaccxyz( sig_acc_bias_i )

                # 5) Gyroscope bias
                sig_gyro_bias_i = np.full((1, 3), np.random.normal(self.sig_gyroscope_bias_mean, self.sig_gyroscope_bias_sig)).flatten()
                self.IMUparams.setsigbiasgyroxyz( sig_gyro_bias_i )

                # Write config files
                self.iSAMConfig.writetojson( path="02_trajectory_estimation/config/SBGsystem/", filename="iSAMConfig_.json" )
                self.IMUparams.writetojson( path="02_trajectory_estimation/config/SBGsystem/", filename="IMUparams_.json" )

                # ############################################################
                # Optimize trajectory using isam2
                #

                start_time = time.time()
                print(f"{start_time} [INFO] started random search iteration: (" + str(i+1) + " / " + str(self.number_samplings) + ")\n" )

                print("SETTINGS: " )
                print( "[INFO] Integration       sigma: {:.8f}".format(sig_int_i[0]) )
                print( "[INFO] Acceleration      sigma: {:.8f}".format(sig_acc_i[0]) )
                print( "[INFO] Gyroscope         sigma: {:.8f}".format(sig_gyro_i[0]) )
                print( "[INFO] Acceleration bias sigma: {:.8f}".format(sig_acc_bias_i[0]) )
                print( "[INFO] Gyroscope bias    sigma: {:.8f}".format(sig_gyro_bias_i[0]) + "\n" )

                print( "OPTIMIZING Trajectory" )

                process = subprocess.Popen(
                    ['sh', "02_trajectory_estimation/run_isam.sh"],
                    env={**os.environ, **shell_in},
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                    )

                # Optional: print to terminal
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
                        + ", {:.8f}".format(sig_int_i[0])
                        + ", {:.8f}".format(sig_acc_i[0])
                        + ", {:.8f}".format(sig_gyro_i[0])
                        + ", {:.8f}".format(sig_acc_bias_i[0])
                        + ", {:.8f}".format(sig_gyro_bias_i[0])
                        + "\n" )
                
                execution_time = time.time() - start_time
                print(f"{time.time()} [INFO] finished evaluation in {execution_time} [s] ")
                print( "RESULTS       ATRtotal: {:.8f}".format(RMStotal) )

    def readdistributionsfromjson(self, path, filename):

        # Construct the full path to the JSON file
        filepath = path + filename

        logger.info( "Reading .json gridsearch lists: " + path + filename )
        
        # Open and read the JSON file
        with open(filepath, 'r') as json_file:
            data = json.load(json_file)

        self.sig_int_mean = data['sig_integration']['mu']
        self.sig_int_sig = data['sig_integration']['sig']

        self.sig_acceleration_mean = data['sigmas_acceleration']['mu']
        self.sig_acceleration_sig = data['sigmas_acceleration']['sig']

        self.sig_gyroscope_mean = data['sigmas_gyroscope']['mu']
        self.sig_gyroscope_sig = data['sigmas_gyroscope']['sig']

        self.sig_acceleration_bias_mean = data['sigmas_acceleration_bias']['mu']
        self.sig_acceleration_bias_sig = data['sigmas_acceleration_bias']['sig']

        self.sig_gyroscope_bias_mean = data['sigmas_gyroscope_bias']['mu']
        self.sig_gyroscope_bias_sig = data['sigmas_gyroscope_bias']['sig']

    def initialize(self, path ):
        self.iSAMConfig.readfromjson( path, "iSAMConfig.json" )
        self.IMUparams.readfromjson( path, "IMUparams.json" )