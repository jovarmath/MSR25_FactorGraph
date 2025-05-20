import numpy as np

import srcpython.geodetictools._01base_classes as baseclasses

import time

class GTSAMdata:
    def __init__(self):

        self.trajectory = baseclasses.Trajectory3D()

    def readTrajectory(self, path: str = "", xyz_frame: str = "", rpy_frame: str = "", read_flg: str = "", offset: np.ndarray = np.array([0,0,0])):
        
        filename_trajectory = "T_graph.trj"
        filename_pose_rpy_cov = "T_rpy_std.txt"
        filename_pose_xyz_cov = "T_xyz_std.txt"
        filename_velo_xyz_cov = "T_vxvyvz_std.txt"
        filename_bias_acc_std = "T_bias_acc_std.txt"
        filename_bias_gyro_std = "T_bias_gyro_std.txt"
        filename_bias_estimates = "T_bias.txt"

        # Reading
        # 1) Trajectory States
        array_data = np.loadtxt( path + filename_trajectory, delimiter=',' )

        # apply global offset
        array_data[:,1] -= offset[0]
        array_data[:,2] -= offset[1]
        array_data[:,3] -= offset[2]

        # 2) State covariance matrices
        pose_rpy_cov = np.loadtxt( path + filename_pose_rpy_cov, delimiter=',' )
        pose_xyz_cov = np.loadtxt( path + filename_pose_xyz_cov, delimiter=',' )
        velo_xyz_cov = np.loadtxt( path + filename_velo_xyz_cov, delimiter=',' )

        # 3) Imu Bias 
        bias_states = np.zeros((1,7))
    

        bias_acc_cov = np.loadtxt( path + filename_bias_acc_std, delimiter=',' )
        bias_gyro_cov = np.loadtxt( path + filename_bias_gyro_std, delimiter=',' )

        if len(bias_acc_cov) > 0:
            self.trajectory.numberofbiasstates = len(bias_states[:,0])
        else:
            self.trajectory.numberofbiasstates = 0

        self.trajectory.xyz_frame = xyz_frame
        self.trajectory.rpy_frame = rpy_frame

        # number of states
        self.trajectory.numberofstates = len(array_data[:,0])
        self.trajectory.time = array_data[:,0]

        # store all states in one matrix, order: time, E, N, H, Vx, Vy, Vz, roll, pitch yaw
        self.trajectory.statesall = array_data

        # Bounding box of the point cloud
        self.trajectory.bbox2D = np.array([ np.min(self.trajectory.statesall[:,1]), np.max(self.trajectory.statesall[:,1]),
                                                np.min(self.trajectory.statesall[:,2]), np.max(self.trajectory.statesall[:,2]) ])
            
        if len(bias_states) > 0:
            self.trajectory.biasstatesall = bias_states
        else:
            self.trajectory.biasstatesall = []

        print("-------------------------------------------------------------------------------")
        print("Gtsam Trajectory Reading Report")
        print( time.strftime("%H:%M:%S"), " filename: " , self.trajectory.statesall[0,:])
        print( time.strftime("%H:%M:%S"), " first state trajectory : " , self.trajectory.statesall[0,:])
        print( time.strftime("%H:%M:%S"), " time interval: " , self.trajectory.time[-1] - self.trajectory.time[0], " [s], [", self.trajectory.time[0], self.trajectory.time[-1], "]")
        print( time.strftime("%H:%M:%S"), " roll pitch yaw frame: ", rpy_frame)
        print( time.strftime("%H:%M:%S"), " xyz frame: ", xyz_frame )
        print( time.strftime("%H:%M:%S"), " number of states ", self.trajectory.statesall.shape )
        print("-------------------------------------------------------------------------------\n")

    def read_GNSS_used( self, path: str = "", offset: np.ndarray = np.array([0,0,0]) ):

        filename_gnss_used = "gnss_used.txt"

        # 4) Used GNSS positions
        used_gnss = np.loadtxt( path + filename_gnss_used, delimiter=',' ) # time, x, y, z
        
        gnss = baseclasses.GNSSdata()
        
        if len(used_gnss.shape) > 1:

            gnss.time = used_gnss[:,0]
            gnss.x = used_gnss[:,1] - offset[0]
            gnss.y = used_gnss[:,2] - offset[1]
            gnss.z = used_gnss[:,3] - offset[2]
        
        else:

            gnss.time = used_gnss[0]
            gnss.x = used_gnss[1] - offset[0]
            gnss.y = used_gnss[2] - offset[1]
            gnss.z = used_gnss[3] - offset[2]

        return gnss



        
