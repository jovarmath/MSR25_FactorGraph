import math
import numpy as np

# Trajectopy
from trajectopy_core.trajectory import Trajectory
from trajectopy_core.evaluation.comparison import compare_trajectories_absolute
from trajectopy_core.matching import match_trajectories

def evaluate_trajectopy_atr( path_T_, path_T_ref, print_to_termial: bool = False   ):

     # Read reference trajectory
    reference_traj_isam = Trajectory.from_file( path_T_ref )

    # Current isam trajectory
    Traj_i_isam = Trajectory.from_file( path_T_ )
    
    # Match trajectories with reference
    match_trajectories( traj_test = Traj_i_isam, traj_ref = reference_traj_isam )
                    
    # Compare to reference
    ATE_isam = compare_trajectories_absolute( traj_test = Traj_i_isam, traj_ref = reference_traj_isam )

    if print_to_termial:
        print("---------------------------------------------------------------------------------------------")
        print("Current trajectory rms to reference")
        print('RMS roll                   ' + str(ATE_isam.property_dict['RMS Roll [°]']) + ' [°]' )
        print('RMS pitch                  ' + str(ATE_isam.property_dict['RMS Pitch [°]']) + ' [°]' )
        print('RMS yaw                    ' + str(ATE_isam.property_dict['RMS Yaw [°]']) + ' [°]' )
        print('RMS along track            ' + str(ATE_isam.property_dict['RMS Along-Track [m]']) + ' [m]' )
        print('RMS horizontal cross track ' + str(ATE_isam.property_dict['RMS Horizontal Cross-Track [m]']) + ' [m]' )
        print('RMS vertical cross track   ' + str(ATE_isam.property_dict['RMS Vertical Cross-Track [m]']) + ' [m]' )
        print("---------------------------------------------------------------------------------------------\n")

     # Orientation deviation @10m distance
    rms_roll_at10m = float(ATE_isam.property_dict['RMS Roll [°]']) * (np.pi/180) * 10
    rms_pitch_at10m =  float(ATE_isam.property_dict['RMS Pitch [°]']) * (np.pi/180) * 10
    rms_yaw_at10m =  float(ATE_isam.property_dict['RMS Yaw [°]']) * (np.pi/180) * 10

    rms_x = float(ATE_isam.property_dict['RMS Along-Track [m]'])
    rms_y = float(ATE_isam.property_dict['RMS Horizontal Cross-Track [m]'])
    rms_z = float(ATE_isam.property_dict['RMS Vertical Cross-Track [m]'])

    values =  [rms_x, rms_y, rms_z, rms_roll_at10m, rms_pitch_at10m, rms_yaw_at10m ]
    RMStotal = calculate_rms(values)

    return ATE_isam.property_dict, RMStotal

def calculate_rms(values):

    # Calculate the sum of squares
    sum_of_squares = sum(x ** 2 for x in values)
    
    # Calculate the mean of the squares
    mean_of_squares = sum_of_squares / len(values)
    
    # Calculate the square root of the mean of the squares
    rms_value = math.sqrt(mean_of_squares)
    
    return rms_value