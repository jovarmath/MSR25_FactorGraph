import numpy as np
import subprocess
import os
import time
from datetime import datetime
from hyperopt import tpe, hp, fmin, Trials
import threading

from srcpython.IMUStochasticParameter import *
from srcpython.iSAMConfig import *
from srcpython.tuningtools.evaluateError import evaluate_error, evaluate_error_prism

from srcpython.tuningtools.gridsearch import *
from srcpython.tuningtools.randomsearch import *

from scipy.spatial.transform import Rotation as R

class iSAMtuning:
    # ______________________________________________________________________________________________________
    # ______________________________________________________________________________________________________
    # Initialization function

    def __init__(self ):

        self.settings_ = { 'PathDATASET': None,
                           'PathCONFIG': None,
                           'PathOUT': None,
                           'PathREF': None,
                           'PathSensorData': None }
        
        self.print_terminal_: bool = False

        # Config files for iSAM
        self.iSAMConfig_ = iSAMConfig()
        self.IMUparams_ = IMUStochasticParameter()

        # Current best loss
        self.best_loss = np.inf
        self.best_params = None

        # Evaluation values
        self.nees_i_ = np.inf
        self.rmse_pos_i = np.inf
        self.rmse_rot_i = np.inf

        self.loss = np.inf

        self.method = None
        self.metric = None

        # Path for reporint the losses of each iteration
        self.report_path: str = None
        self.reportfile = None

        # Counter to report the number of calls for evaluating the objective function
        self.obj_call_counter = 0

    # ______________________________________________________________________________________________________
    # ______________________________________________________________________________________________________
    # Loss evaluation function
    
    def evaluate_loss( self, params ):
        
        # Set stochastic parameter
        self.set_stochastic_parameter( params )

        # Optimize trajectory
        returncode = self.optimize_trajectory( print_isam_terminalout = self.print_terminal_ )

        # Compute error metrics
        if returncode == 1:
            
            # If referene poses are used for tuning
            if self.metric == "nees" or self.metric == "pos" or self.metric == "rot" :
                print("Metric: ", self.metric ) 

                returns_ = evaluate_error( self.settings_["PathOUT"], self.settings_["PathREF"], self.tau  )
                    
                # Update nees, rmse rot and pos
                self.nees_i_, self.rmse_pos_i, self.rmse_rot_i = returns_['NEES_sum'], returns_['Pos_rmse'], returns_['Rot_rmse']
                    
                # Set the loss
                if self.metric == 'nees':
                    self.loss = self.nees_i_
                elif self.metric == 'pos':
                    self.loss = self.rmse_pos_i
                elif self.metric == 'rot':
                    self.loss = self.rmse_rot_i

            # If prism positions are used for tuning
            elif self.metric == "pos_prism":
                print("Metric: ", self.metric )

                returns_ = evaluate_error_prism( self.settings_["PathOUT"], self.settings_["PathSensorData"], self.tau )

                # Set loss
                self.loss = returns_['Pos_rmse']


        # If ISAM2 fails due to wrong tuning parameters 
        else:
            self.nees_i_, self.rmse_pos_i, self.rmse_rot_i, self.loss = np.inf, np.inf, np.inf, np.inf

        # Write report line for current iteration
        self.write_report_i()
            
        # Check if loss is smaller
        if self.loss < self.best_loss:
            self.best_loss = self.loss
            self.best_params = params

            # Write best parameter to file
            self.IMUparams_.writetojson( path = "results/smbo/BestSettingsTmp/", filename = 'IMUparams.json' )

        # Print best loss and parameter
        self.print_results()

        # Update counter
        self.obj_call_counter += 1

        # Return loss
        return self.loss

    # Loss evaluation initial
    #
    def evaluate_loss_initial( self ):
        
        # Set stochastic parameter to default values
        self.read_initial_config()

        # Optimize trajectory
        self.optimize_trajectory()

        # Compute NEES (Normalized estimation error squared)
        returns_ = evaluate_error( self.settings_["PathOUT"], self.settings_["PathREF"] )

        self.nees_i_, self.rmse_pos_i, self.rmse_rot_i = returns_['NEES_sum'], returns_['Pos_rmse'], returns_['Rot_rmse'] 

        return self.nees_i_

    def print_loss(self, RMStotal):

        P = (
            f"\n ______________________________________________________\n"
            f"| ----------------------- LOSS INFO -------------------- |\n"
            f"| Current loss:  "+"{:.6f}".format(RMStotal) + " [m] \n" 
            f"| Best loss  "+"{:.6f}".format(self.best_loss) +" [m] \n" 
            f"| _______________________________________________________|\n" )

        print(P)

    def print_results(self):

        if 'sig_acceleration_bias' in self.best_params:
            sigaccbias_si = self.best_params['sig_acceleration_bias']
            sigaccbias_ug = sigaccbias_si / (10**-6 * 9.80665)
        else:
            sigaccbias_si = 0
            sigaccbias_ug = 0
        
        if 'sig_gyroscope_bias' in self.best_params:
            siggyrobias_si = self.best_params['sig_gyroscope_bias']
            siggyrobias_deg_hr = siggyrobias_si / ((np.pi/180) / 3600)
        else:
            sigaccbias_si = 0
            sigaccbias_ug = 0 

        if 'sig_integration' in self.best_params:
            sigintegration = self.best_params['sig_integration']
        else:
            sigintegration = 0

        if self.metric == 'nees':
            P = (
                f"\n ______________________________________________________\n"
                f"| ------------- Current Result INFO -------------------- |\n"
                f"| Iteration:            "+"{:.0f}".format(self.obj_call_counter) + "\n"
                f"| Current loss (NEES):  "+"{:.6f}".format(self.loss) + " [-] \n" 
                f"| Best loss (NEES):     "+"{:.6f}".format(self.best_loss) +" [-] \n"
                f"| Best integration sig: "+"{:.6f}".format(sigintegration) +"[-] \n"
                f"| Best sig acc bias:    "+"{:.6f}".format(sigaccbias_si) +"[m/s^2], "+"{:.2f}".format(sigaccbias_ug) +" [ug] \n"
                f"| Best sig gyro bias:   "+"{:.6f}".format(siggyrobias_si) +" [rad/s], "+"{:.2f}".format(siggyrobias_deg_hr) +"[°/hr] \n"    
                f"| _______________________________________________________|\n" )
            
        elif self.metric == 'pos':
            P = (
                f"\n ______________________________________________________\n"
                f"| ------------- Current Result INFO -------------------- |\n"
                f"| Iteration:            "+"{:.0f}".format(self.obj_call_counter) + "\n"
                f"| Current loss (Pos):   "+"{:.6f}".format(self.loss) + " [m] \n" 
                f"| Best loss (Pos):     "+"{:.6f}".format(self.best_loss) +" [m] \n"
                f"| Best integration sig: "+"{:.6f}".format(sigintegration) +"[-] \n"
                f"| Best sig acc bias:    "+"{:.6f}".format(sigaccbias_si) +"[m/s^2], "+"{:.2f}".format(sigaccbias_ug) +" [ug] \n"
                f"| Best sig gyro bias:   "+"{:.6f}".format(siggyrobias_si) +" [rad/s], "+"{:.2f}".format(siggyrobias_deg_hr) +"[°/hr] \n"    
                f"| _______________________________________________________|\n" )

        elif self.metric == 'rot':
            P = (
                f"\n ______________________________________________________\n"
                f"| ------------- Current Result INFO -------------------- |\n"
                f"| Iteration:            "+"{:.0f}".format(self.obj_call_counter) + "\n"
                f"| Current loss (Rot):   "+"{:.6f}".format(self.loss) + " [deg] \n" 
                f"| Best loss (Rot):     "+"{:.6f}".format(self.best_loss) +" [deg] \n"
                f"| Best integration sig: "+"{:.6f}".format(sigintegration) +"[-] \n"
                f"| Best sig acc bias:    "+"{:.6f}".format(sigaccbias_si) +"[m/s^2], "+"{:.2f}".format(sigaccbias_ug) +" [ug] \n"
                f"| Best sig gyro bias:   "+"{:.6f}".format(siggyrobias_si) +" [rad/s], "+"{:.2f}".format(siggyrobias_deg_hr) +"[°/hr] \n"    
                f"| _______________________________________________________|\n" )
            
        elif self.metric == 'pos_prism':
            P = (
                f"\n ______________________________________________________\n"
                f"| ------------- Current Result INFO -------------------- |\n"
                f"| Iteration:                  "+"{:.0f}".format(self.obj_call_counter) + "\n"
                f"| Current loss (Pos Prism):   "+"{:.6f}".format(self.loss) + " [m] \n" 
                f"| Best loss (Pos):            "+"{:.6f}".format(self.best_loss) +" [m] \n"
                f"| Best integration sig:       "+"{:.6f}".format(sigintegration) +"[-] \n"
                f"| Best sig acc bias:          "+"{:.6f}".format(sigaccbias_si) +"[m/s^2], "+"{:.2f}".format(sigaccbias_ug) +" [ug] \n"
                f"| Best sig gyro bias:         "+"{:.6f}".format(siggyrobias_si) +" [rad/s], "+"{:.2f}".format(siggyrobias_deg_hr) +"[°/hr] \n"    
                f"| _______________________________________________________|\n" )

        print(P)    
    
    def read_initial_config(self):
        self.iSAMConfig_.readfromjson(self.settings_['PathDATASET'] + "config/", filename = "iSAMConfig.json" ) 
        self.IMUparams_.readfromjson(self.settings_['PathDATASET'] + "config/", filename = "IMUparams.json" )

    def read_config(self, path):
        self.iSAMConfig_.readfromjson( path, filename = "iSAMConfig.json" ) 
        self.IMUparams_.readfromjson( path, filename = "IMUparams.json" )        

    def set_settings(self, PathDATASET, PathCONFIG, PathOUT, PathREF, PATH_SENSOR_DATA):
        self.settings_ = { 'PathDATASET': PathDATASET,
                           'PathCONFIG': PathCONFIG,
                           'PathOUT': PathOUT,
                           'PathREF': PathREF,
                           'PathSensorData': PATH_SENSOR_DATA }
    
    def set_stochastic_parameter(self, params, path = None):

        print(params)

        # Set preintegration sigma
        if 'sig_integration' in params:
            self.iSAMConfig_.set_preintegrationSig( np.full( (1, 3), params['sig_integration'] ).flatten() )
                                
        # Set acceleration sigma
        if 'sig_acceleration' in params:
            self.IMUparams_.setsigaccxyz( np.full( (1, 3), params['sig_acceleration'] ).flatten() ) 

        # Set gyroscope sigma
        if 'sig_gyroscope' in params:
            self.IMUparams_.setsiggyroxyz( np.full( (1, 3), params['sig_gyroscope'] ).flatten() ) 

        # Set acceleration bias sigma
        if 'sig_acceleration_bias' in params:
            self.IMUparams_.setsigbiasaccxyz( np.full( (1, 3), params['sig_acceleration_bias'] ).flatten() )
        
        # Set gyroscope bias sigma
        if 'sig_gyroscope_bias' in params:    
            self.IMUparams_.setsigbiasgyroxyz( np.full( (1, 3), params['sig_gyroscope_bias'] ).flatten() ) 

        # Write to .jsons
        #
        if path is None:
            self.iSAMConfig_.writetojson( self.settings_["PathCONFIG"], "iSAMConfig.json" )
            self.IMUparams_.writetojson( self.settings_["PathCONFIG"], "IMUparams.json" )
        else:
            self.iSAMConfig_.writetojson( path, "iSAMConfig.json" )
            self.IMUparams_.writetojson( path, "IMUparams.json" )

    def write_report_i(self):

        print( "[INFO] Writing loss report" )
        self.reportfile.write( "{:.0f}".format(self.obj_call_counter)
                           + ", {:.8f}".format(self.nees_i_)
                           + ", {:.8f}".format(self.rmse_pos_i)
                           + ", {:.8f}".format(self.rmse_rot_i)
                           + ", {:.8f}".format(self.iSAMConfig_.preintegrationSig[0])
                           + ", {:.8f}".format(self.IMUparams_.sigaccx)
                           + ", {:.8f}".format(self.IMUparams_.siggyrox)
                           + ", {:.8f}".format(self.IMUparams_.sigbiasaccx)
                           + ", {:.8f}".format(self.IMUparams_.sigbiasgyrox)
                           + "\n" )
        
    def write_report_initial(self):

        current_time = datetime.now()
        ATRoutpath = "results/initial/" + current_time.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs( ATRoutpath )

        file = open( ATRoutpath + "/Err.txt", 'w', encoding='utf-8')    
        file.write( "# NEES [-], RMSE pos [m], RMSE rot [°], SigAcc [m/s^2], SigGyro [rad/s], SigBiasAcc [m/s^2], SigBiasGyro [rad/s] \n")

        print( "[INFO] Write initial report file" )

        file.write(  "{:.8f}".format(self.nees_i_)
                   + ", {:.8f}".format(self.rmse_pos_i)
                   + ", {:.8f}".format(self.rmse_rot_i)
                   + ", {:.8f}".format(self.iSAMConfig_.preintegrationSig[0])
                   + ", {:.8f}".format(self.IMUparams_.sigaccx)
                   + ", {:.8f}".format(self.IMUparams_.siggyrox)
                   + ", {:.8f}".format(self.IMUparams_.sigbiasaccx)
                   + ", {:.8f}".format(self.IMUparams_.sigbiasgyrox)
                   + "\n" )
        
        file.close()


    def get_best_parameter_from_report_file(self, path):

        resultfile = np.loadtxt(fname = path + "Err.txt", delimiter=",", comments="#")

        # Find minimum loss index
        minidx = np.argmin( resultfile[:,1] )

        return { 'sig_integration': resultfile[minidx,4],
                 'sig_acceleration': resultfile[minidx,5],
                 'sig_gyroscope': resultfile[minidx,6],
                 'sig_acceleration_bias': resultfile[minidx,7],
                 'sig_gyroscope_bias': resultfile[minidx,8] }

    def optimize_trajectory(self, print_isam_terminalout = False):

        print( "[INFO] Optimizing trajectory" )

        print(self.settings_)

        process = subprocess.Popen( ['sh', "02_trajectory_estimation/run_isam.sh"],
                                    env={**os.environ, **self.settings_},
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                    text=True )
        
        # Start a thread that will kill the process after 120 seconds
        #timeout_thread = threading.Thread(target=self.kill_process_after_timeout, args=(process))
        #timeout_thread.start()

        stdout_data = []
        stderr_data = []
                
        if print_isam_terminalout:     
            # Print output in real-time
            for stdout_line in iter(process.stdout.readline, ''):
                print(stdout_line, end='')
                stdout_data.append(stdout_line)

            for stderr_line in iter(process.stderr.readline, ''):
                print(stderr_line, end='')
                stderr_data.append(stderr_line)

            process.stdout.close()
            process.stderr.close()

        returncode = process.wait()

        if not print_isam_terminalout:
            stdout_data, stderr_data = process.communicate()

        stdout = ''.join(stdout_data)
        stderr = ''.join(stderr_data)

        # Check if the process was killed due to timeout
        if returncode is None:
            stderr += "\nProcess terminated due to timeout" 

        return returncode

    
    # Function to kill the process after the timeout
    def kill_process_after_timeout(p):
        timeout = 900 # 15 min
        time.sleep(timeout) 
        if p.poll() is None:  # Process is still running
            print("[ERROR] Optimization process exceeded the time limit of 2 minutes and was terminated.")
            p.kill()

    # ##############################################################################################
    # INITIAL LOSS
    # ##############################################################################################

    def calcualte_initial_loss( self, tau ):

        # Set initialization time
        self.tau = tau

        # Set path to data and config files
        self.set_settings( PathDATASET = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/",
                              PathCONFIG = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/config/",
                              PathOUT = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/initial/",
                              PathREF = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/reference/" )
    
        # Read initial config files
        self.read_initial_config()
        
        # Set parameters
        params = { 'sig_integration': self.iSAMConfig_.preintegrationSig[0],
                   'sig_acceleration': self.IMUparams_.sigaccx,
                   'sig_gyroscope': self.IMUparams_.siggyrox,
                   'sig_acceleration_bias': self.IMUparams_.sigbiasaccx,
                   'sig_gyroscope_bias': self.IMUparams_.sigbiasgyrox }

        # Evaluate current loss
        NEES = self.evaluate_loss( params )

        # Write results to file
        self.write_report_initial()

    # ##############################################################################################
    # METHODS
    
    # ##############################################################################################
    # GRIDSEARCH
    # ##############################################################################################

    def run_gridsearch( self ):

        self.set_settings( PathDATASET = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/",
                           PathCONFIG = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/config/gridsearch/",
                           PathOUT = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/gridsearch/",
                           PathREF = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/reference/"  )
        
        # Read initial config files
        self.read_initial_config()

        # Create output path and create reportfile
        current_time = datetime.now()
        self.report_path = "results/gridsearch/" + current_time.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs( self.report_path )
        
        self.reportfile = open( self.report_path + "/Err.txt", 'w', encoding='utf-8')    
        self.reportfile.write( "# NEES [-], RMSE pos [m], RMSE rot [°], SigAcc [m/s^2], SigGyro [rad/s], SigBiasAcc [m/s^2], SigBiasGyro [rad/s] \n")

        GridSearch = gridsearch()
        GridSearch.readlistsfromjson( path = "data/", filename="gridsearchlists.json" )

        # Total number od iterations
        number_of_iterations = len(GridSearch.sig_preintegration) * len(GridSearch.sig_acceleration) * len(GridSearch.sig_gyroscope) * len(GridSearch.sig_acceleration_bias) * len(GridSearch.sig_gyroscope_bias)
        it_cnt = 0

        # Loop over integration sigma
        for i in np.arange(0, len(GridSearch.sig_preintegration) ):
                
            # Loop over acceleration sigma
            for j in np.arange(0, len(GridSearch.sig_acceleration) ):
                    
                # Loop over gyroscope sigma
                for k in np.arange(0, len(GridSearch.sig_gyroscope) ):

                     # Loop over acceleration bias sigma
                    for l in np.arange(0, len(GridSearch.sig_acceleration_bias) ):
                                
                        # Loop over gyroscope bias sigma
                        for m in np.arange(0, len(GridSearch.sig_gyroscope_bias) ):

                            # Set parameters
                            params = { 'sig_integration': GridSearch.sig_preintegration[i][0],
                                       'sig_acceleration': GridSearch.sig_acceleration[j][0],
                                       'sig_gyroscope': GridSearch.sig_gyroscope[k][0],
                                       'sig_acceleration_bias': GridSearch.sig_acceleration_bias[l][0],
                                       'sig_gyroscope_bias': GridSearch.sig_gyroscope_bias[m][0]
                            }

                            # Print progress
                            P = (
                                    f"\n ____________________________________________________\n"
                                    f"| --------------- GRID search iteration ------------ |\n"
                                    f"| "+ str(it_cnt+1) + " / " + str(number_of_iterations)+"\n"
                                    f"|____________________________________________________|\n" )
                            
                            print(P)
                            print(params)

                            # Evaluate current loss
                            NEES = self.evaluate_loss( params )

                            # Check if best loss
                            if NEES < self.best_loss:
                                self.best_loss = NEES

                            # Write report for current estimate
                            self.write_report_i()

                            # Update iteration counter
                            it_cnt += 1
        # Close report file
        self.reportfile.close()

    # ##############################################################################################
    # RANDOMSEARCH
    # ##############################################################################################

    def run_randomsearch(self, settings ):

        self.set_settings( PathDATASET = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/",
                           PathCONFIG = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/config/randomsearch/",
                           PathOUT = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/randomsearch/",
                           PathREF = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/reference/"  )

        # Read initial config files
        self.read_initial_config()

        # Create output path and create reportfile
        current_time = datetime.now()
        self.report_path = "results/randomsearch/" + current_time.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs( self.report_path )
        
        self.reportfile = open( self.report_path + "/Err.txt", 'w', encoding='utf-8')    
        self.reportfile.write( "# NEES [-], RMSE pos [m], RMSE rot [°], SigAcc [m/s^2], SigGyro [rad/s], SigBiasAcc [m/s^2], SigBiasGyro [rad/s] \n")

        RandomSearch = randomsearch()
        RandomSearch.readdistributionsfromjson( path = "data/", filename="randomsearchconfig.json" )
        RandomSearch.number_samplings = settings['number_iterations']

        for i in np.arange(0, RandomSearch.number_samplings ): 
                                            
            # Set parameters
            params = { 'sig_integration': np.random.normal(RandomSearch.sig_int_mean, RandomSearch.sig_int_sig),
                       'sig_acceleration': np.random.normal(RandomSearch.sig_acceleration_mean, RandomSearch.sig_acceleration_sig),
                       'sig_gyroscope': np.random.normal(RandomSearch.sig_gyroscope_mean, RandomSearch.sig_gyroscope_sig),
                       'sig_acceleration_bias': np.random.normal(RandomSearch.sig_acceleration_bias_mean, RandomSearch.sig_acceleration_bias_sig),
                       'sig_gyroscope_bias': np.random.normal(RandomSearch.sig_gyroscope_bias_mean, RandomSearch.sig_gyroscope_bias_sig)
                      }
            
            # Print progress
            P = (
                f"\n ____________________________________________________\n"
                f"| ------------ RANDOM search iteration ------------- |\n"
                f"| "+ str(i+1) + " / " + str(RandomSearch.number_samplings)+"\n"
                f"|____________________________________________________|\n" )
                            
            print(P)
            print(params)

            # Evaluate current loss
            NEES = self.evaluate_loss( params )

            # Check if best loss
            if NEES < self.best_loss:
                self.best_loss = NEES
                self.best_params = params

            # Print current result
            self.print_loss( NEES )

            # Write report for current estimate
            self.write_report_i()
            
        # Close report file
        self.reportfile.close()

        print("Finished randomsearch in", current_time - datetime.now())

    # ##############################################################################################
    # Sequential Model-Based Optimization
    # ##############################################################################################

    def run_smbo( self, SET ):

        self.set_settings( PathDATASET = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/",
                           PathCONFIG = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/config/smbo/",
                           PathOUT = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/smbo/",
                           PathREF = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/02_trajectory/reference/",
                           PATH_SENSOR_DATA = "/mnt/d/trajectory-estimation-data/sbg-data/24_07_22/03/01_sensordata/" )
        
        # Read initial config files
        self.read_initial_config()

        self.tau = SET['tau']

        # Create output path and create reportfile
        current_time = datetime.now()
        self.report_path = "results/smbo/" + current_time.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs( self.report_path )
        
        self.reportfile = open( self.report_path + "/Err.txt", 'w', encoding='utf-8')    
        self.reportfile.write( "# NEES [-], RMSE pos [m], RMSE rot [°], SigAcc [m/s^2], SigGyro [rad/s], SigBiasAcc [m/s^2], SigBiasGyro [rad/s] \n")

        # ###################################################################################
        # Set parameter distribution
        
        # Normal distribution

        if SET['distribution'] == 'normal':
            param_space = { 'sig_integration': hp.normal('sig_integration', 0, 0.02),
                            'sig_acceleration': hp.normal('sig_acceleration', 0.003952579, 0.0002),
                            'sig_gyroscope': hp.normal('sig_gyroscope', 0.00030855, 0.00002),
                            'sig_acceleration_bias': hp.normal('sig_acceleration_bias', 0.0001372931, 0.00001),
                            'sig_gyroscope_bias': hp.normal('sig_gyroscope_bias', 0.000033937, 0.001)
            }
        elif SET['distribution'] == 'uniform':
            param_space = { 'sig_integration': hp.uniform('sig_integration', 0, 0.1),
                            'sig_acceleration': hp.uniform('sig_acceleration', 0.001, 0.01),
                            'sig_gyroscope': hp.uniform('sig_gyroscope', 0.0001, 0.001),
                            'sig_acceleration_bias': hp.uniform('sig_acceleration_bias', 0, 0.00029420), # = Interval [8 - 30 ug], SBG: 14 ug
                            'sig_gyroscope_bias': hp.uniform('sig_gyroscope_bias', 0, 0.00048481) # = Interval [0 - 100 °/hr], SBG: 7 °/hr
            }
        elif SET['distribution'] == 'lognormal':
            param_space = { 'sig_integration': hp.lognormal('sig_integration', 0.000001, 0.1),
                            'sig_acceleration': hp.lognormal('sig_acceleration', 0.001, 0.01),
                            'sig_gyroscope': hp.lognormal('sig_gyroscope', 0.0001, 0.001),
                            'sig_acceleration_bias': hp.lognormal('sig_acceleration_bias', 0.000015, 0.001),
                            'sig_gyroscope_bias': hp.lognormal('sig_gyroscope_bias', 0.00000001, 0.1)
            }

        # Choose just the parameters that should be optimized
        param_space = {key: value for key, value, flag in zip(param_space.keys(), param_space.values(), SET['optimize']) if flag}

        print(param_space)

        trials = Trials()

        best = fmin(
            fn=self.evaluate_loss, # Objective Function to optimize
            space=param_space, # Hyperparameter's Search Space
            algo=tpe.suggest, # Optimization algorithm (representative TPE)
            max_evals=SET["number_iterations"], # Number of optimization attempts
            trials=trials
        )

        print("Best parameter: ", best)

        # close report file
        self.reportfile.close()
