import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

from geodetictools._01base import RotmatX, RotmatY, RotmatZ


def evaluate_error( path_T_, path_T_ref, tau ):

    print( "[INFO] Evaluating trajectory error" )

    # #############################################################################################
    # 1) READ Trajectories and covariances matrices
    # 

    #fields t,px,py,pz,vx,vy,vz,ex,ey,ez
    T_i = np.loadtxt( fname = path_T_ + "T_graph.trj", comments="#", delimiter="," )
    Cov_rpy = np.loadtxt( fname = path_T_ + "T_pose_std.txt", comments="#", delimiter="," )

    #fields t,l,px,py,pz,qx,qy,qz,qw,vx,vy,vz
    T_ref = np.loadtxt( fname = path_T_ref + "T_graph.traj", comments="#", delimiter="," )

    # #############################################################################################
    # 2) Intersect Trajectories
    # 
    
    # Cut the initialization phase from trajectory
    T_i = T_i[T_i[:, 0] >= T_i[0,0] + tau]

    # Intersect
    xy, x_ind, y_ind = np.intersect1d( T_i[:,0], T_ref[:,0], return_indices=True )

    T_ref_ = T_ref[y_ind]
    T_i_ = T_i[x_ind]

    # Intersect covariances
    Cov_rpy_ = Cov_rpy[x_ind]

    # NEES
    nees_ = np.zeros(xy.shape)

    # Rotation error
    rot_err_ = np.zeros(xy.shape)
    yaw_err_ = np.zeros(xy.shape)
    pitch_err_ = np.zeros(xy.shape)
    roll_err_ = np.zeros(xy.shape)

    x_err_ = np.zeros(xy.shape)
    y_err_ = np.zeros(xy.shape)
    z_err_ = np.zeros(xy.shape)

    # Position error
    p_err_ = np.zeros(xy.shape)

    # Compute NEES for each state
    for i in np.arange(0,xy.shape[0]):
        
        # 1) Rotation error

        # Rotation reference
        r_ref = R.from_quat( T_ref_[i,5:9] )
        R_ref = r_ref.as_matrix()

        # Rotation current state
        r_cur = R.from_euler( 'xyz', T_i_[i,7:11], degrees=False ) 
        R_cur = r_cur.as_matrix()
        
        # Error in rotation
        r_err = R.from_matrix(np.dot(R_ref.T, R_cur))
        euler_err_ = r_err.as_euler("xyz", degrees=False)

        # Rotation error
        rot_err_[i] = np.sqrt( euler_err_[0]**2 + euler_err_[1]**2 + euler_err_[2]**2 )
        yaw_err_[i] = euler_err_[2] #Yaw
        pitch_err_[i] = euler_err_[1] #Pitch
        roll_err_[i] = euler_err_[0] #Roll

        # 2) XYZ error
        xyz_err_ = T_ref_[i,2:5] - T_i_[i,1:4]

        x_err_[i] = xyz_err_[0]
        y_err_[i] = xyz_err_[1]
        z_err_[i] = xyz_err_[2]

        # position
        p_err_[i] = np.sqrt( xyz_err_[0]**2 + xyz_err_[1]**2 + xyz_err_[2]**2 )
        
        # Error as vector
        exk = np.concatenate((euler_err_, xyz_err_), axis=0)
        
        # Covariance
        use_correlations = False
    
        cov_array1d = Cov_rpy_[i,1:] # covariance matrix as vector
        Sig = cov_array1d.reshape((6,6)) # covariance matrix as matrix

        # Set up inverse covariance matrix
        if use_correlations:
            Sigma_inv = np.linalg.inv(Sig)
        else:
            Sig_zeroed = np.zeros_like(Sig)
            np.fill_diagonal(Sig_zeroed, np.diag(Sig))
            Sigma_inv = np.linalg.inv(Sig_zeroed)

        # estimation error squared
        nees_[i] = exk.T @ Sigma_inv @ exk

    nees_sum_ = np.sum(nees_)
    xyz_err_mean_ = np.mean(p_err_)
    rot_err_mean_ = np.degrees( np.mean(rot_err_) )

    # Return values
    returns_ = { 'NEES': nees_,
                 'Pos_err': p_err_,
                 'Rot_err': rot_err_,
                 'X Error': x_err_,
                 'Y Error': y_err_,
                 'Z Error': z_err_,
                 'Yaw Error': yaw_err_,
                 'Pitch Error': pitch_err_,
                 'Roll Error': roll_err_,
                 'NEES_sum': nees_sum_,
                 'Pos_rmse': xyz_err_mean_,
                 'Rot_rmse': rot_err_mean_,
                 'trajectory': T_i_,
                 'trajectory_ref': T_ref_ }
    
    return returns_



def evaluate_error_prism( path_T_, path_ref_prism, tau ):

    # ___________________________________________________________________________
    # Read trajectory from file

    # fields t,px,py,pz,vx,vy,vz,ex,ey,ez
    T_i = np.loadtxt( fname = path_T_ + "T_graph.trj", comments="#", delimiter="," )

    # Cut the initialization phase from trajectory
    T_i = T_i[T_i[:, 0] >= T_i[0,0] + tau]

    # ___________________________________________________________________________
    # Read trajectory from file

    #fields t,px,py,pz,state
    P_i = np.loadtxt( fname = path_ref_prism + "TS_data.txt" )
    
    # ___________________________________________________________________________
    # Intersect trajectory with prism positions

    xy, x_ind, y_ind = np.intersect1d( T_i[:,0], P_i[:,0], return_indices=True )
    T_i = T_i[x_ind, :]
    P_i = P_i[y_ind, :]

    # ___________________________________________________________________________
    # Leverarm body --> prism
    lv = np.array( [-0.480550, 0.015950, -0.31918] )
    
    # ___________________________________________________________________________
    # Loop over all poses

    x_err_ = np.zeros(xy.shape)
    y_err_ = np.zeros(xy.shape)
    z_err_ = np.zeros(xy.shape)

    p_err_ = np.zeros(xy.shape)

    for i in np.arange(len(T_i[:,0])):

        # Position
        xyz = np.array([T_i[i,1], T_i[i,2], T_i[i,3]])
        
        # Rotation
        R = np.dot(np.dot(RotmatZ( T_i[i,9] ), RotmatY( T_i[i,8] )), RotmatX( T_i[i,7] ))
        
        # _________________________________________________________
        # Prism position

        xyz_prism_pred = xyz + np.dot( R, lv )

        # _________________________________________________________
        # Errors

        # xyz
        xyz_err_ = P_i[i,1:4] - xyz_prism_pred

        # Error per direction
        x_err_[i] = xyz_err_[0]
        y_err_[i] = xyz_err_[1]
        z_err_[i] = xyz_err_[2]

        # 3D position
        p_err_[i] = np.sqrt( xyz_err_[0]**2 + xyz_err_[1]**2 + xyz_err_[2]**2 )

    xyz_err_mean_ = np.mean(p_err_)

    # Return values
    returns_ = { 'NEES': 0.0,
                 'Pos_err': p_err_,
                 'Rot_err': 0.0,
                 'X Error': x_err_,
                 'Y Error': y_err_,
                 'Z Error': z_err_,
                 'Yaw Error': 0.0,
                 'Pitch Error': 0.0,
                 'Roll Error': 0.0,
                 'NEES_sum': 0.0,
                 'Pos_rmse': xyz_err_mean_,
                 'Rot_rmse': 0.0,
                 'trajectory': None,
                 'trajectory_ref': None }
    
    return returns_