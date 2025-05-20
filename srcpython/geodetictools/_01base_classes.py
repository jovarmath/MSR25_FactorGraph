from asyncio import TimerHandle
from os import times
import numpy as np
from numpy.core.numeric import roll
from scipy.interpolate import interp1d
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import pandas as pd
import glob
import os
import time

# Own classes and functions
import srcpython.geodetictools._03CoordinateTransformations as geotransform
import srcpython.geodetictools._01base as geobase
from srcpython.geodetictools._04Visualization import PlotTabs

# Table of Content
# (1) Pose 3D class
#   (1.1) settime(self, time, time_frame)
#   (1.2) setrpy(self, rpy, frame)
#   (1.3) setxyz(self, xyz, frame)
#   (1.4) plot2D

# (2) Trajectory3D class
#   (2.1) functions
#         - interpolate (self, time, kind )
#         - plot2D(self, steps)

# ====================================================================
# Pose 3D Class
# ====================================================================     
# 
class Pose3:
    def __init__(self, xyz, rpy = None, R = None ):

        # window_title: str = "PlotTabs"

        # xyz
        self.x = xyz[0]
        self.y = xyz[1]
        self.z = xyz[2]

        if rpy is not None:
            # Euler angles
            self.roll = rpy[0]
            self.pitch = rpy[1]
            self.yaw = rpy[2]

            # Rotation matrix
            self.R = geobase.RotmatZ( self.yaw ) @ geobase.RotmatY( self.pitch ) @ geobase.RotmatX( self.roll )

        if R is not None:
            self.R = R

            # Rotation matrix to euler angles
            rpy = geobase.Rotmat2Euler( self.R )

            # Euler angles
            self.roll = rpy[0]
            self.pitch = rpy[1]
            self.yaw = rpy[2]

        # Transformation matrix
        T = np.identity(4)
        T[:3, :3] = self.R
        T[:3, 3] = np.array([self.x, self.y, self.z])
        self.T = T
    #######################################################################################################################################

    def inverse( self ):
        return Pose3( xyz = np.array( [-self.x, -self.y, -self.z] ), R = self.R.T )
    
    #######################################################################################################################################

    def settime(self,time,time_frame):
        self.time=time
        self.time_frame = time_frame

    #######################################################################################################################################

    def setrpy(self,r,p,y,rpy_frame):
        self.roll=r
        self.pitch=p
        self.yaw=y
        self.rpy_frame=rpy_frame

    #######################################################################################################################################

    def setxyz(self,x,y,z,xyz_frame):
        self.x=x
        self.y=y
        self.z=z
        self.xyz_frame=xyz_frame

    #######################################################################################################################################

    def plot2D( self, ax, axis_length = 0.2 ):

            xyz = geobase.vector3(self.x, self.y, self.z)

            x_axis = xyz + self.R[0,:] * axis_length
            y_axis = xyz + self.R[1,:] * axis_length

            x_X = [ xyz[0], x_axis[0] ]
            x_Y = [ xyz[1], x_axis[1] ]

            y_X = [ xyz[0], y_axis[0] ]
            y_Y = [ xyz[1], y_axis[1] ]

            ax.plot(x_X, x_Y, "-r", linewidth=3)
            ax.plot(y_X, y_Y, "-g", linewidth=3)

    #######################################################################################################################################

    def plot3D( self, ax, axis_length = 0.2):

        xyz = geobase.vector3(self.x, self.y, self.z)

        x_axis = (xyz + self.R[0,:] * axis_length).flatten()
        y_axis = (xyz + self.R[1,:] * axis_length).flatten()
        z_axis = (xyz + self.R[2,:] * axis_length).flatten()

        x_X = [ xyz[0], x_axis[0] ]
        x_Y = [ xyz[1], x_axis[1] ]
        x_Z = [ xyz[2], x_axis[2] ]

        y_X = [ xyz[0], y_axis[0] ]
        y_Y = [ xyz[1], y_axis[1] ]
        y_Z = [ xyz[2], y_axis[2] ]

        z_X = [ xyz[0], z_axis[0] ]
        z_Y = [ xyz[1], z_axis[1] ]
        z_Z = [ xyz[2], z_axis[2] ]

        ax.plot3D( x_X, x_Y, x_Z, "-r" )
        ax.plot3D( y_X, y_Y, y_Z, "-g" )
        ax.plot3D( z_X, z_Y, z_Z, "-b" )

#######################################################################################################################################

# ====================================================================
# Trajectory 3D class
# ====================================================================

#######################################################################################################################################

class Trajectory3D:

    def __init__(self, numberofstates: int = 0,
                       numberofbiasstates: int = 0,
                       time: int = [],
                       time_frame: str = [],
                       xyz_frame: str = [],
                       rpy_frame: str = [] ) -> None:
        
        # 1) General Trajectory Info
        self.numberofstates = numberofstates
        self.numberofbiasstates = numberofbiasstates
        self.time = time
        self.time_frame = time_frame
        self.xyz_frame = xyz_frame
        self.rpy_frame = rpy_frame

        # All states [time, xyz, velocity, rpy, bias acceleration, bias gyroscope] 16
        self.statesall = np.zeros( (numberofstates, 16) , dtype=float)
        self.bbox2D = np.array([0,0,0,0]) # Xmin, Xmax, Ymin, Ymax

        # Standard Deviation
        #self.statesall_sigma = np.zeros( (numberofstates,10) , dtype=float)

        # 2) Rotation class rotation()
        #self.rotation = [rotation()] * numberofstates # list of rpy
        # rotation attributes:
        # time
        # frame
        # roll
        # pitch
        # yaw
        # covariance

        # 3) Translation class translation()
        #self.translation = [translation()] * numberofstates
        # translation attributes:
        # time
        # frame
        # x
        # y
        # z
        # covariances

        # 3) Translation class translation()
        #self.velocity = [velocity()] * numberofstates
        # translation attributes:
        # time
        # frame
        # velocity x
        # velocity y
        # velocity z
        # covariances

    #######################################################################################################################################

    def append_pose(self, 
                    time: float = [],
                    xyz: np.ndarray = np.empty((0,3)),
                    vxvyvz: np.ndarray= np.empty((0,3)),
                    rpy: np.ndarray= np.empty((0,3))) -> None:

        line = np.c_[time, xyz[0], xyz[1], xyz[2], vxvyvz[0], vxvyvz[1], vxvyvz[2], rpy[0], rpy[1], rpy[2], np.zeros( (1,6)) ]

        self.statesall = np.concatenate((self.statesall, line), axis=0)

    #######################################################################################################################################

    def delete_data(self):

        self.numberofstates = 0
        self.numberofbiasstates = 0
        self.time = []
        self.time_frame = []
        self.xyz_frame = []
        self.rpy_frame = []

        # All states [time, xyz, velocity, rpy, bias acceleration, bias gyroscope] 16
        self.statesall = np.zeros( (self.numberofstates, 16) , dtype=float)

    #######################################################################################################################################

    def interpolate(self, timestamps, kind = "cubic"):
        
        print("-------------------------------------------------------------------------------")
        print("Interpolating Trajectory")
        print( time.strftime("%H:%M:%S"), "Number of states before interpolation", self.statesall.shape )
        print( time.strftime("%H:%M:%S"), "New timestamps ", np.min(timestamps), np.max(timestamps) )
        print( time.strftime("%H:%M:%S"), "Own timestamps ", np.min(self.time), np.max(self.time) )

        # Find & delete duplicate values in time vector
        idx__ = geobase.find_duplicate( self.statesall[:,0] )
        self.statesall = np.delete( self.statesall, idx__, axis=0)
        self.numberofstates = len( self.statesall[:,0] )
        self.time = np.delete( self.time, idx__, axis=0)

        print( time.strftime("%H:%M:%S"), "Duplicate indices in time vector ", idx__ )
        
        # Translation
        f_X = interp1d(self.statesall[:,0], self.statesall[:,1], kind=kind, fill_value="extrapolate") # X ECEF
        f_Y = interp1d(self.statesall[:,0], self.statesall[:,2], kind=kind, fill_value="extrapolate") # Y ECEF
        f_Z = interp1d(self.statesall[:,0], self.statesall[:,3], kind=kind, fill_value="extrapolate") # Z ECEF

        # find intersecting interval of data
        X_int = f_X( timestamps )
        Y_int = f_Y( timestamps )
        Z_int = f_Z( timestamps )

        # Velocity
        f_VX = interp1d(self.statesall[:,0], self.statesall[:,4], kind=kind, fill_value="extrapolate") # X velo
        f_VY = interp1d(self.statesall[:,0], self.statesall[:,5], kind=kind, fill_value="extrapolate") # Y velo
        f_VZ = interp1d(self.statesall[:,0], self.statesall[:,6], kind=kind, fill_value="extrapolate") # Z velo

        # find intersecting interval of data
        VX_int = f_VX( timestamps )
        VY_int = f_VY( timestamps )
        VZ_int = f_VZ( timestamps )

        # Euler Angles To Rotation structure 
        r = R.from_euler('xyz', np.c_[self.statesall[:,7], self.statesall[:,8], self.statesall[:,9]], degrees=False)
        
        # Slerp interpolation, as geodetic curve on unit sphere
        slerp = Slerp(self.statesall[:,0], r)

        interp_rots = slerp( timestamps )

        # Rotation to Euler Angles
        rpy_inter = interp_rots.as_euler('xyz', degrees=False)

        roll_int  = rpy_inter[:,0] # roll
        pitch_int = rpy_inter[:,1] # pitch
        yaw_int   = rpy_inter[:,2] # yaw

        # Fill new Trajectory with data
        Tr_interpolated = Trajectory3D()
        
        # All states [time, xyz, velocity, rpy, bias acceleration, bias gyroscope] 16
        Tr_interpolated.statesall = np.c_[timestamps, X_int, Y_int, Z_int, VX_int, VY_int, VZ_int, roll_int, pitch_int, yaw_int, np.zeros( (len(yaw_int),6)) ]

        Tr_interpolated.rotation = [ rotation() ] * len(roll_int)
        Tr_interpolated.translation = [ translation() ] * len(roll_int)
        Tr_interpolated.velocity = [ velocity() ] * len(roll_int)
        
        Tr_interpolated.x=X_int
        Tr_interpolated.y=Y_int
        Tr_interpolated.z=Z_int
        Tr_interpolated.xyz_frame = self.xyz_frame
        Tr_interpolated.rpy_frame = self.rpy_frame

        # Trajectory Data
        Tr_interpolated.numberofstates = len(roll_int)
        Tr_interpolated.time = timestamps

        Tr_interpolated.bbox2D = np.array([np.min( Tr_interpolated.statesall[:,1] ),
                                           np.max( Tr_interpolated.statesall[:,1] ),
                                           np.min( Tr_interpolated.statesall[:,2] ),
                                           np.max( Tr_interpolated.statesall[:,2] )])
        
        print( time.strftime("%H:%M:%S"), "Number of states after interpolation", Tr_interpolated.statesall.shape )
        print("-------------------------------------------------------------------------------\n")

        return Tr_interpolated

    #######################################################################################################################################

    def crop(self, interval, method = "time"):
        
        print("-------------------------------------------------------------------------------")
        print("Trajectory Cropping")
        print(time.strftime("%H:%M:%S"), "number before:", self.statesall.shape)
        
        if method == "time":
            
            # Determin indices
            idx = np.where( (self.statesall[:,0] > interval[0]) & (self.statesall[:,0] < interval[1]) )
            idx = idx[0]
            
            self.statesall = self.statesall[idx,:]
            self.time = self.time[idx]
            self.numberofstates = len(idx)

            self.bbox2D = np.array([np.min( self.statesall[:,1] ),
                                    np.max( self.statesall[:,1] ),
                                    np.min( self.statesall[:,2] ),
                                    np.max( self.statesall[:,2] )])
            
        print(time.strftime("%H:%M:%S"), "number after:", self.statesall.shape)    
        print("-------------------------------------------------------------------------------\n")

    #######################################################################################################################################
        
    def crop_by_index( self, idx ):
                
        print("-------------------------------------------------------------------------------")
        print("Trajectory Index Cropping")
        print(time.strftime("%H:%M:%S"), "number before:", self.statesall.shape)

        T_ = Trajectory3D()

        T_.statesall = self.statesall[ idx,: ]
        T_.time = T_.statesall[:,0]
        T_.numberofstates = len( idx )

        T_.bbox2D = np.array([np.min( T_.statesall[:,1] ),
                              np.max( T_.statesall[:,1] ),
                              np.min( T_.statesall[:,2] ),
                              np.max( T_.statesall[:,2] )])

        print(time.strftime("%H:%M:%S"), "number after:", T_.statesall.shape)
        print("--------------------------------------------------------------------------------\n")

        return T_
    

    #######################################################################################################################################

    def compute_bbox2D(self):

        self.bbox2D = np.array([np.min( self.statesall[:,1] ),
                                np.max( self.statesall[:,1] ),
                                np.min( self.statesall[:,2] ),
                                np.max( self.statesall[:,2] )])
        
    #######################################################################################################################################

    def rpy_toENU(self):
        if self.rpy_frame == "NED":
            print("[INFO] Orientation Angle Transformation: From NED to ENU Frame")
            R_ENU_NED = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
            R_NED_ENU = R_ENU_NED.T
                
            print("- [INFO] First Angles before Tranform, RPY ", self.statesall[0,7], " ", self.statesall[0,8], " ", self.statesall[0,9] )

            for i in np.arange( 0, len(self.statesall[:,9]) ):

                R_BN = R_NED_ENU @ geobase.RotmatZ(self.statesall[i,9]) @ geobase.RotmatY(self.statesall[i,8]) @ geobase.RotmatX(self.statesall[i,7])

                rpy = geobase.Rotmat2Euler( R_BN )

                #self.rotation[i].yaw = rpy[2]
                #self.rotation[i].pitch = rpy[1]
                #self.rotation[i].roll = rpy[0]

                #self.rotation[i].frame = target_system

                self.statesall[i,7] = rpy[0]
                self.statesall[i,8] = rpy[1]
                self.statesall[i,9] = rpy[2]

            self.rpy_frame = 'ENU'

            print("- [INFO] Changed RPY Frame to ENU Frame")
            print("- [INFO] First Angles after Transform, RPY ", self.statesall[i,7], " ", self.statesall[i,8], " ", self.statesall[i,9])
        else:
            print("- [INFO] RPY Frame already ENU Frame")

    def rpy_toNED(self):
        if self.rpy_frame == "ENU":
            print("[INFO] Orientation Angle Transformation: From ENU to NED Frame")
            R_ENU_NED = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
                
            print("- [INFO] First Angles before Tranform, RPY ", self.statesall[0,7], " ", self.statesall[0,8], " ", self.statesall[0,9] )

            for i in np.arange( 0, len(self.statesall[:,9]) ):

                R_BN = R_ENU_NED @ geobase.RotmatZ(self.statesall[i,9]) @ geobase.RotmatY(self.statesall[i,8]) @ geobase.RotmatX(self.statesall[i,7])

                rpy = geobase.Rotmat2Euler( R_BN )

                #self.rotation[i].yaw = rpy[2]
                #self.rotation[i].pitch = rpy[1]
                #self.rotation[i].roll = rpy[0]

                #self.rotation[i].frame = target_system

                self.statesall[i,7] = rpy[0]
                self.statesall[i,8] = rpy[1]
                self.statesall[i,9] = rpy[2]

            self.rpy_frame = 'NED'

            print("- [INFO] Changed RPY Frame to NED Frame")
            print("- [INFO] First Angles after Transform, RPY ", self.statesall[i,7], " ", self.statesall[i,8], " ", self.statesall[i,9])
        else:
            print("- [INFO] RPY Frame already NED Frame")

    # OLD:
    # def rpy_transform(self, target_system):

    #     print("--------------------------------------------------------------------------------")
    #     if self.rpy_frame == "END" and target_system == "NED":
    #         print("[INFO] Orientation Angle Transformation: From ", self.rpy_frame, " to ", target_system , " Frame")

    #         R_END_ENU = geobase.RotmatZ( geobase.deg2rad( -90 ) ) @ geobase.RotmatY( geobase.deg2rad( 180 ) )
            
    #         print("- [INFO] First Angles before Tranform, RPY ", self.statesall[0,7], " ", self.statesall[0,8], " ", self.statesall[0,9] )

    #         for i in np.arange( 0, self.numberofstates ):

    #             R_BN = R_END_ENU @ geobase.RotmatZ(self.statesall[i,9]) @ geobase.RotmatY(self.statesall[i,8]) @ geobase.RotmatX(self.statesall[i,7])

    #             rpy = geobase.Rotmat2Euler( R_BN )

    #             #self.rotation[i].yaw = rpy[2]
    #             #self.rotation[i].pitch = rpy[1]
    #             #self.rotation[i].roll = rpy[0]

    #             #self.rotation[i].frame = target_system

    #             # TODO: Change all states!!! check
    #             self.statesall[i,7] = rpy[0]
    #             self.statesall[i,8] = rpy[1]
    #             self.statesall[i,9] = rpy[2]

    #         self.rpy_frame = target_system

    #         print("- [INFO] Changed RPY Frame to ", target_system, " Frame")
    #         print("- [INFO] First Angles after Transform, RPY ", self.statesall[i,7], " ", self.statesall[i,8], " ", self.statesall[i,9])

    #######################################################################################################################################

    def xyz_transform(self, target_system):

        print("--------------------------------------------------------------------------------")
        print("[INFO] Translation Transformation: From ", self.xyz_frame, " to ", target_system , " Frame")
        if self.xyz_frame == "UTM" and target_system == "ECEF":
            for i in np.arange( 0, self.numberofstates ):
                lat, lon = geotransform.utm2ell(self.translation[i].y,self.translation[i].x,Zone=32)


                x,y,z = geotransform.ell2xyz(lat,lon,self.translation[i].z)

                self.translation[i].x = x
                self.translation[i].y = y
                self.translation[i].z = z

                self.statesall[i,1] = x
                self.statesall[i,2] = y
                self.statesall[i,3] = z

            self.xyz_frame = target_system
            print("- [INFO] Changed XYZ Frame to ", target_system, " Frame")
            print("- [INFO] First Translation after Transform, xyz ", self.translation[0].x, " ", self.translation[0].y, " ", self.translation[0].z)
            print("--------------------------------------------------------------------------------")
    
    #######################################################################################################################################

    def intersect(self, timestamps_new):

        print("--------------------------------------------------------------------------------")
        print( time.strftime("%H:%M:%S")," Intersecting trajectory with time stamps " )
        print( time.strftime("%H:%M:%S"), " Length before: ", len(self.time) )

        x_s = np.max([self.time[0], timestamps_new[0]]) # start index
        x_e = np.min([self.time[-1], timestamps_new[-1]]) # end index

        print( time.strftime("%H:%M:%S"), " Time start: ", x_s )
        print( time.strftime("%H:%M:%S"), " Time end: ", x_e )

        x_new_1 = np.argwhere( (self.time < x_s) & (self.time > x_e ))

        self.time = np.delete(self.time, x_new_1.flatten())
        self.statesall = np.delete(self.statesall, x_new_1.flatten(), axis=0)

        print( time.strftime("%H:%M:%S"), " Length after: ", len(self.time) )
        print("--------------------------------------------------------------------------------")

    #######################################################################################################################################

    def write_to_file(self, path, filename, offsxyz = np.array([0,0,0])):
            
            # Create Folder
            if not os.path.exists( path ):
                os.makedirs( path )

            # add global offset
            self.statesall[:,1] += offsxyz[0]
            self.statesall[:,2] += offsxyz[1]
            self.statesall[:,3] += offsxyz[2]

            print( time.strftime("%H:%M:%S"), " write trajectory: #T = ", len(self.statesall[:,0]) )

            df = pd.DataFrame(data = self.statesall[0:-1,0:10])
            df.to_csv( path + filename + ".trj", sep=' ', header=False, float_format='%.8f', index=False)

            with open(path + filename + ".trj", 'r+') as file:
                content = file.read()
                file.seek(0, 0)  # Setze den Schreibkopf an den Anfang der Datei
                file.write("#name sbgekf\n")
                file.write("#epsg 25832\n")
                file.write("#fields t,px,py,pz,vx,vy,vz,ex,ey,ez\n")
                file.write(content)

            #name sbgekf
            #epsg 25832
            #fields t,px,py,pz,vx,vy,vz,ex,ey,ez

            # remove global offset
            self.statesall[:,1] -= offsxyz[0]
            self.statesall[:,2] -= offsxyz[1]
            self.statesall[:,3] -= offsxyz[2]

    #######################################################################################################################################

    # =====================================================================================================================================
    # PLOT Functions
    # =====================================================================================================================================

    #######################################################################################################################################

    def plot_bbox2D( self ):

        x = [self.bbox2D[0], self.bbox2D[0], self.bbox2D[1], self.bbox2D[1], self.bbox2D[0]]
        y = [self.bbox2D[2], self.bbox2D[3], self.bbox2D[3], self.bbox2D[2], self.bbox2D[2]]
        
        plt.fill(x, y, 'r', alpha=0.6)



    def plot_compare_to(self, T2, ts_data = [], gnss_data = []):

        # Visualize gtsam trajectory state estimates
        tabs = PlotTabs(window_title="Trajectory Compare Viewer")

        # #################################################
        # Figure 1: XY
        FZ = 16
        fig1 = plt.figure()
        plt.plot(self.statesall[:,1], self.statesall[:,2], ".k", markersize=4)
        plt.plot(T2.statesall[:,1], T2.statesall[:,2], ".r", markersize=4)

        plt.xlim((np.min(self.statesall[:,1]), np.max(self.statesall[:,1])))
        plt.ylim((np.min(self.statesall[:,2]), np.max(self.statesall[:,2])))
        plt.axis("equal")

        plt.grid("on")
        plt.xlabel("UTM East", fontsize=FZ )
        plt.ylabel("UTM North", fontsize=FZ )

        # #################################################
        # Figure 2: Heights
        fig2 = plt.figure()
        plt.plot( self.statesall[:,0], self.statesall[:,3], ".k", markersize=4, label="SBG EKF" )
        plt.plot( T2.statesall[:,0], T2.statesall[:,3], ".r", markersize=4, label="Trajectory (TS & IMU)" )
        plt.plot( gnss_data.time, gnss_data.z, ".b", label="GNSS positions")
        plt.plot( ts_data.time, ts_data.z, ".y", label="TS positions")

        plt.legend()

        plt.grid("on")
        plt.xlabel("UTM East", fontsize=FZ )
        plt.ylabel("UTM North", fontsize=FZ )

        # #################################################
        # Figure 3: Roll Pitch Yaw
        fig3, (ax1, ax2, ax3) = plt.subplots(3)
        fig3.suptitle('Roll Pitch Yaw', fontsize=FZ)
        ax1.plot(self.statesall[:,0], self.statesall[:,7] * (180 / np.pi), "-r", linewidth=3, label="SBG EKF")
        ax1.plot(T2.statesall[:,0], T2.statesall[:,7] * (180 / np.pi), ".k", label="Own")

        ax2.plot(self.statesall[:,0], self.statesall[:,8] * (180 / np.pi), "-g", linewidth=3, label="SBG EKF")
        ax2.plot(T2.statesall[:,0], T2.statesall[:,8] * (180 / np.pi), ".k", label="Own")

        ax3.plot(self.statesall[:,0], self.statesall[:,9] * (180 / np.pi), "-b", linewidth=3, label="SBG EKF")
        ax3.plot(T2.statesall[:,0], T2.statesall[:,9] * (180 / np.pi), ".k", label="Own")

        tabs.addPlot(title= "XY",figure=fig1 )
        tabs.addPlot(title= "Z", figure=fig2 )
        tabs.addPlot(title= "RPY", figure=fig3)
        tabs.show()

    #######################################################################################################################################

    def plot2D( self, steps, axis_length=0.5, gnss_data = [], ts_data = [], number_poses = False, ownfigure = True ):

        FZ = 20

        if ownfigure:
            fig, ax = plt.subplots(figsize=(10, 6), layout='constrained')

        if ts_data:
            plt.plot(ts_data.x, ts_data.y, ".b", markersize=4 )

        if gnss_data:
            plt.plot(gnss_data.x, gnss_data.y, ".k", markersize=4 )

        # Loop over all states
        for i in np.arange( 0, self.numberofstates , steps ):
            
            # Define Pose3 
            Posei = Pose3( xyz = np.array([self.statesall[i,1], self.statesall[i,2], self.statesall[i,3]]),
                           rpy = np.array([self.statesall[i,7], self.statesall[i,8], self.statesall[i,9]]) )

            Posei.plot2D( ax, axis_length=axis_length )

            if number_poses:
                plt.text( self.statesall[i,1], self.statesall[i,2], str(i) )

        plt.grid("on")
        plt.xlabel("UTM East [m]", fontsize=FZ )
        plt.ylabel("UTM North [m]", fontsize=FZ )

        if ts_data:
            plt.legend(["TS"])

        if gnss_data:
            plt.legend(["GNSS"])
            
        plt.xlim((np.min(self.statesall[:,1]), np.max(self.statesall[:,1])))
        plt.ylim((np.min(self.statesall[:,2]), np.max(self.statesall[:,2])))
        plt.axis("equal")

        return fig
        
    #######################################################################################################################################

    def plot3D(self, steps, axis_length ):

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        # Loop over all states
        for i in np.arange( 0, self.numberofstates , steps ):

            if self.xyz_frame == "ECEF":

                # ECEF to LLA

                (lat, lon, alt) = geotransform.ecef2lla(self.translation[i].x, self.translation[i].y, self.translation[i].z, ellipsoid="GRS80", version=1)


                # LLA to UTM

                N,E = geotransform.ell2utm(lat,lon,Zone=32)

                # Define Pose3 
                Posei = Pose3(time=self.time[i], time_frame="GPST",
                            x=self.statesall[i,1], y=self.statesall[i,2], z=self.statesall[i,3], xyz_frame="UTM", 
                            roll=self.statesall[i,7], pitch=self.statesall[i,8], yaw=self.statesall[i,9], rpy_frame=self.rpy_frame)


                Posei.plot3D( axis_length=axis_length, ax=ax)
            
            elif self.xyz_frame == "UTM":

                # Define Pose3 
                Posei = Pose3(time=self.time[i], time_frame="GPST",
                            x=self.statesall[i,1], y=self.statesall[i,2], z=self.statesall[i,3], xyz_frame="UTM", 
                            roll=self.statesall[i,7], pitch=self.statesall[i,8], yaw=self.statesall[i,9], rpy_frame=self.rpy_frame)


                Posei.plot3D(axis_length=axis_length, ax=ax)

        ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])

        return ax
    
    #######################################################################################################################################

    def plotZ( self, ts_data = [] ):

        FZ = 16

        fig, ax = plt.subplots()
        plt.plot( self.statesall[:,0], self.statesall[:,3], '.k')

        if ts_data:
            plt.plot( ts_data.time, ts_data.z, '.r')


        plt.grid("on")
        plt.xlabel("Time", fontsize=FZ)
        plt.ylabel("ellip. height [m]", fontsize=FZ)
        plt.xlim([self.time[0], self.time[-1]])
        plt.title("Trajectory Height Profile", fontsize=FZ)
        ax.tick_params(axis='both', which='major', labelsize=FZ)

        return fig

    #######################################################################################################################################

    def plotVelocity( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        fig.suptitle('Velocity', fontsize=FZ)
        ax1.plot(self.statesall[:,0], self.statesall[:,4], ".r")
        ax2.plot(self.statesall[:,0], self.statesall[:,5], ".g")
        ax3.plot(self.statesall[:,0], self.statesall[:,6], ".b")

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlim([self.statesall[0,0], self.statesall[-1,0]])
        ax2.set_xlim([self.statesall[0,0], self.statesall[-1,0]])
        ax3.set_xlim([self.statesall[0,0], self.statesall[-1,0]])

        ax1.set_ylabel("vx [m/s]", fontsize=FZ)
        ax2.set_ylabel("vy [m/s]", fontsize=FZ)
        ax3.set_ylabel("vz [m/s]", fontsize=FZ)

        ax3.set_xlabel("time [s]", fontsize=FZ)

        return fig

    #######################################################################################################################################

    def plotRollPitchYaw( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        fig.suptitle('Roll Pitch Yaw')
        ax1.plot(self.statesall[:,0], self.statesall[:,7], ".r")
        ax2.plot(self.statesall[:,0], self.statesall[:,8], ".g")
        ax3.plot(self.statesall[:,0], self.statesall[:,9] , ".b")
        
        ax1.set_ylabel("roll [deg]", fontsize=FZ)
        ax2.set_ylabel("pitch [deg]", fontsize=FZ)
        ax3.set_ylabel("yaw [deg]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlim((self.statesall[0,0], self.statesall[-1,0]))
        ax2.set_xlim((self.statesall[0,0], self.statesall[-1,0]))
        ax3.set_xlim((self.statesall[0,0], self.statesall[-1,0]))

        ax3.set_xlabel("time [s]", fontsize=FZ)

        return fig
    
    #######################################################################################################################################

    def plotGyroBias( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)

        if hasattr(self, 'biasstatesall'):

            fig.suptitle('Gyro Bias Estimates')
            ax1.plot(self.biasstatesall[:,0], self.biasstatesall[:,4], ".r")
            ax2.plot(self.biasstatesall[:,0], self.biasstatesall[:,5], ".g")
            ax3.plot(self.biasstatesall[:,0], self.biasstatesall[:,6], ".b")

            ax1.set_ylabel("bwx [rad/s]", fontsize=FZ)
            ax2.set_ylabel("bwy [rad/s]", fontsize=FZ)
            ax3.set_ylabel("bwz [rad/s]", fontsize=FZ)
            
            ax1.tick_params(axis='both', which='major', labelsize=FZ)
            ax2.tick_params(axis='both', which='major', labelsize=FZ)
            ax3.tick_params(axis='both', which='major', labelsize=FZ)

            ax1.set_xlim([self.biasstatesall[0,0], self.biasstatesall[-1,0]])
            ax2.set_xlim([self.biasstatesall[0,0], self.biasstatesall[-1,0]])
            ax3.set_xlim([self.biasstatesall[0,0], self.biasstatesall[-1,0]])

            ax3.set_xlabel("time [s]", fontsize=FZ)

        else:
            fig.suptitle('No Data Available')

        return fig
    
    #######################################################################################################################################

    def plotAccelerationBias( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        
        if hasattr(self, 'biasstatesall'):

            fig.suptitle('Acceleration Bias Estimates')
            ax1.plot(self.biasstatesall[:,0], self.biasstatesall[:,1], ".r")
            ax2.plot(self.biasstatesall[:,0], self.biasstatesall[:,2], ".g")
            ax3.plot(self.biasstatesall[:,0], self.biasstatesall[:,3], ".b")

            ax1.set_ylabel("bax [m/s^2]", fontsize=FZ)
            ax2.set_ylabel("bay [m/s^2]", fontsize=FZ)
            ax3.set_ylabel("baz [m/s^2]", fontsize=FZ)
            
            ax1.tick_params(axis='both', which='major', labelsize=FZ)
            ax2.tick_params(axis='both', which='major', labelsize=FZ)
            ax3.tick_params(axis='both', which='major', labelsize=FZ)

            ax1.set_xlim([self.biasstatesall[0,0], self.biasstatesall[-1,0]])
            ax2.set_xlim([self.biasstatesall[0,0], self.biasstatesall[-1,0]])
            ax3.set_xlim([self.biasstatesall[0,0], self.biasstatesall[-1,0]])

            ax3.set_xlabel("time [s]", fontsize=FZ)

        else:
            fig.suptitle('No Data Available')

        return fig
    

    #######################################################################################################################################

    def plotRollPitchYaw_sig( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        fig.suptitle("Standard Deviations Roll Pitch Yaw")
        ax1.plot(self.statesall_sigma[:,0], geobase.rad2deg( self.statesall_sigma[:,7] ), ".r")
        ax2.plot(self.statesall_sigma[:,0], geobase.rad2deg( self.statesall_sigma[:,8] ), ".g")
        ax3.plot(self.statesall_sigma[:,0], geobase.rad2deg( self.statesall_sigma[:,9] ), ".b")
        
        ax1.set_ylabel("sigma roll [deg]", fontsize=FZ)
        ax2.set_ylabel("sigma pitch [deg]", fontsize=FZ)
        ax3.set_ylabel("sigma yaw [deg]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))
        ax2.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))
        ax3.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))

        ax3.set_xlabel("time [s]", fontsize=FZ)

        return fig
    
    #######################################################################################################################################

    def plotXYZ_sig( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        fig.suptitle('Standard Deviations XYZ')
        ax1.plot(self.statesall_sigma[:,0], self.statesall_sigma[:,1], ".r")
        ax2.plot(self.statesall_sigma[:,0], self.statesall_sigma[:,2], ".g")
        ax3.plot(self.statesall_sigma[:,0], self.statesall_sigma[:,3], ".b")
        
        ax1.set_ylabel("sigma east [m]", fontsize=FZ)
        ax2.set_ylabel("sigma north [m]", fontsize=FZ)
        ax3.set_ylabel("sigma altitude [m]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))
        ax2.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))
        ax3.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))

        ax3.set_xlabel("time [s]", fontsize=FZ)

        return fig
    
    #######################################################################################################################################

    def plot_vxvyvz_sig( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)
        fig.suptitle('Standard Deviations Velocity')
        ax1.plot(self.statesall_sigma[:,0], self.statesall_sigma[:,4], ".r")
        ax2.plot(self.statesall_sigma[:,0], self.statesall_sigma[:,5], ".g")
        ax3.plot(self.statesall_sigma[:,0], self.statesall_sigma[:,6], ".b")
        
        ax1.set_ylabel("sigma velocity east [m/s]", fontsize=FZ)
        ax2.set_ylabel("sigma velocity north [m/s]", fontsize=FZ)
        ax3.set_ylabel("sigma velocity altitude [m/s]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))
        ax2.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))
        ax3.set_xlim((self.statesall_sigma[0,0], self.statesall_sigma[-1,0]))

        ax3.set_xlabel("time [s]", fontsize=FZ)

        return fig
    
    #######################################################################################################################################

    def plot_multi(self, gnss_used, ts_data):

        # Visualize gtsam trajectory state estimates
        tabs = PlotTabs(window_title="GTSAM Graph Trajectory Data Viewer")

        steps = 50

        fig1 = self.plot2D( steps=steps, gnss_data=gnss_used, ts_data=ts_data)
        fig2 = self.plotZ( ts_data )
        fig3 = self.plotVelocity()
        fig4 = self.plotRollPitchYaw()
        #fig5 = self.plotAccelerationBias()
        #fig6 = self.plotGyroBias()
        fig7 = ts_data.plot_xy()


        tabs.addPlot(title= "Gtsam Trajectory: Position and Orientation", figure=fig1 )
        tabs.addPlot(title= "Gtsam Trajectory: Heights", figure=fig2 ) 
        tabs.addPlot(title= "Gtsam Trajectory: Velocity", figure=fig3 )
        tabs.addPlot(title= "Gtsam Trajectory: Roll Pitch Yaw", figure=fig4 )
        #tabs.addPlot(title= "Gtsam Trajectory: Accelerometer Bias", figure=fig5 )
        #tabs.addPlot(title= "Gtsam Trajectory: Gyro Bias", figure=fig6 )
        tabs.addPlot(title= "Total Station Data", figure=fig7 )
        
        tabs.show()

        plt.close()



# Data Classes
class rotation:
    def __init__(self, time=0,
                       frame="",
                       rpy= geobase.vector3(0,0,0),
                       cov = geobase.matrix33(0,0,0,0,0,0,0,0,0)):
        
        self.time = time # vector 1
        self.frame = frame
        self.roll = rpy[0] # geobase.vector3(0,0,0)
        self.pitch = rpy[1] # geobase.vector3(0,0,0)
        self.yaw = rpy[2] # geobase.vector3(0,0,0)
        self.cov = cov # geobase.matrix33(0,0,0,0,0,0,0,0,0)

class translation:
    def __init__(self, time=0,
                       frame="",
                       xyz= geobase.vector3(0,0,0),
                       cov = geobase.matrix33(0,0,0,0,0,0,0,0,0)):
        
        self.time = time # vector 1
        self.frame = frame
        self.x = xyz[0] # geobase.vector3(0,0,0)
        self.y = xyz[1] # geobase.vector3(0,0,0)
        self.z = xyz[2] # geobase.vector3(0,0,0)
        self.cov = cov # geobase.matrix33(0,0,0,0,0,0,0,0,0)

class imubias:
    def __init__(self,
                 time=0,
                 biasa = geobase.vector3(0,0,0),
                 biasg = geobase.vector3(0,0,0),
                 biasgcov = geobase.matrix33(0,0,0,0,0,0,0,0,0),
                 biasacov = geobase.matrix33(0,0,0,0,0,0,0,0,0)):
        self.time = time = time # vector nx1
        self.biasg = biasg # geobase.vector3(0,0,0)
        self.biasa = biasa # geobase.vector3(0,0,0)
        self.biasgcov = biasgcov # geobase.matrix33(0,0,0,0,0,0,0,0,0)
        self.biasacov = biasacov # geobase.matrix33(0,0,0,0,0,0,0,0,0)

class velocity:
    def __init__(self, time=0,
                       frame="",
                       V= geobase.vector3(0,0,0),
                       cov = geobase.matrix33(0,0,0,0,0,0,0,0,0)):
        self.time = time # vector 1
        self.frame = frame
        self.x = V[0] # geobase.vector3(0,0,0)
        self.y = V[1] # geobase.vector3(0,0,0)
        self.z = V[2] # geobase.vector3(0,0,0)
        self.cov = cov # geobase.matrix33(0,0,0,0,0,0,0,0,0)



# ############################################################### #
# MEASUREMENT CLASSES
# ############################################################### #

class IMUdata:
    def __init__(self, numberofstates=0, time=0.0, time_frame="undefined", 
                       accx=np.empty( (), dtype= float ), accy=np.empty( (), dtype= float ), accz=np.empty( (), dtype= float ), acc_frame="undefined",
                       gyrox=np.empty( (), dtype= float ), gyroy=np.empty( (), dtype= float ), gyroz=np.empty( (), dtype= float ), gyro_frame="undefined"):

        self.time = time = time
        self.time_frame = time_frame

        self.accx = accx
        self.accy = accy
        self.accz = accz
        self.acc_frame = acc_frame

        # NED or ENU

        self.gyrox = gyrox
        self.gyroy = gyroy 
        self.gyroz = gyroz
        self.gyro_frame = gyro_frame

    def writetofile( self, type, filename ):

        print("--------------------------------------------------------------------------------")
        print("Writing IMU data to file")
        print(time.strftime("%H:%M:%S")," filename: ", filename)
        print(time.strftime("%H:%M:%S")," number: ", len(self.accx))
        
        if (type == "gtsam"):

            data = np.c_[ self.time, self.accx, self.accy, self.accz, self.gyrox, self.gyroy, self.gyroz ]

            np.savetxt(fname=filename, X=data, delimiter=" ", fmt="%10.10f")

        print("--------------------------------------------------------------------------------\n")

    def cut_by_Index(self,idx):
        self.time=self.time[idx]

        self.accx = self.accx[idx]
        self.accy = self.accy[idx]
        self.accz = self.accz[idx]

        self.gyrox = self.gyrox[idx]
        self.gyroy = self.gyroy[idx]
        self.gyroz = self.gyroz[idx]

    def plotAcceleration( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)

        ax1.set_title('IMU Accelerations Measurments', fontsize=FZ)

        ax1.plot(self.time, self.accx, ".r")
        ax2.plot(self.time, self.accy, ".g")
        ax3.plot(self.time, self.accz, ".b")

        ax1.grid("on")
        ax2.grid("on")
        ax3.grid("on")

        ax1.set_ylabel("ax [m/s^2]", fontsize=FZ)
        ax2.set_ylabel("ay [m/s^2]", fontsize=FZ)
        ax3.set_ylabel("az [m/s^2]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlabel("time [s]", fontsize=FZ)
        ax2.set_xlabel("time [s]", fontsize=FZ)
        ax3.set_xlabel("time [s]", fontsize=FZ)

        ax1.set_xlim([self.time[0], self.time[-1]])
        ax2.set_xlim([self.time[0], self.time[-1]])
        ax3.set_xlim([self.time[0], self.time[-1]])

        return fig
    
    def plotAngularvelocity( self ):

        FZ = 16

        fig, (ax1, ax2, ax3) = plt.subplots(3)

        ax1.set_title('IMU Angular Velocity Measurements', fontsize=FZ)

        ax1.plot(self.time, self.gyrox, ".r")
        ax2.plot(self.time, self.gyroy, ".g")
        ax3.plot(self.time, self.gyroz, ".b")

        ax1.grid("on")
        ax2.grid("on")
        ax3.grid("on")

        ax1.set_ylabel("ax [rad/s]", fontsize=FZ)
        ax2.set_ylabel("ay [rad/s]", fontsize=FZ)
        ax3.set_ylabel("az [rad/s]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlabel("time [s]", fontsize=FZ)
        ax2.set_xlabel("time [s]", fontsize=FZ)
        ax3.set_xlabel("time [s]", fontsize=FZ)

        ax1.set_xlim([self.time[0], self.time[-1]])
        ax2.set_xlim([self.time[0], self.time[-1]])
        ax3.set_xlim([self.time[0], self.time[-1]])

        return fig

# =========================================================================================================== #


class GNSSdata:
    def __init__(self, numberofstates=0, time=0.0, time_frame="undefined", 
                    x=np.empty( (), dtype= float ), y=np.empty( (), dtype= float ), z=np.empty( (), dtype= float ), xyz_frame="undefined",
                    sx=np.empty( (), dtype= float ), sy=np.empty( (), dtype= float ), sz=np.empty( (), dtype= float ), Und=np.empty( (), dtype= float ), 
                    NSv = np.empty( (), dtype= float ), DAge = np.empty( (), dtype= float ), State = np.empty( (), dtype= float ) ):

        # Time
        self.time=time
        self.time_frame = time_frame
            
        # Acceleration
        self.x=x
        self.y=y
        self.z=z
        self.xyz_frame = xyz_frame
        self.sx=sx
        self.sy=sy
        self.sz=sz

        # Geoid Undulation, Number of Sat., DAge
        self.Und=Und
        self.NSv=NSv
        self.DAge=DAge
        self.State=State

    def readfile(self, path):

        gnssdata = np.loadtxt( path, delimiter=' ' )

        self.time = gnssdata[:,0]
        self.x = gnssdata[:,1]
        self.y = gnssdata[:,2]
        self.z = gnssdata[:,3]

        self.sx = gnssdata[:,4]
        self.sy = gnssdata[:,5]
        self.sz = gnssdata[:,6]

    def utm2topocentric(self):
        
        # Transform coorinates to local topocentric
        [xyz, T] = geotransform.utm2topocentric( self.x, self.y, self.z )

        # Overwrite
        self.x, self.y, self.z = xyz[:,0], xyz[:,1], xyz[:,2]

        return xyz, T

    def deltebyindices(self, idx_del):

        # Index vector
        idxall = np.arange(0,len(self.time))
        idx = np.delete(idxall, idx_del)

        newGNSSdata = GNSSdata()

        # Time
        newGNSSdata.time = self.time[idx]
        newGNSSdata.time_frame = self.time_frame
            
        # Acceleration
        newGNSSdata.x=self.x[idx]
        newGNSSdata.y=self.y[idx]
        newGNSSdata.z=self.z[idx]
        newGNSSdata.xyz_frame = self.xyz_frame
        newGNSSdata.sx=self.sx[idx]
        newGNSSdata.sy=self.sy[idx]
        newGNSSdata.sz=self.sz[idx]

        # Geoid Undulation, Number of Sat., DAge
        newGNSSdata.Und=self.Und[idx]
        newGNSSdata.NSv=self.NSv[idx]
        newGNSSdata.DAge=self.DAge[idx]
        newGNSSdata.State=self.State[idx]

        return newGNSSdata
    
    def cut_by_Index(self,idx):
        self.time=self.time[idx]
        self.x = self.x[idx]
        self.y = self.y[idx]
        self.z = self.z[idx]

        self.sx= self.sx[idx]
        self.sy= self.sy[idx]
        self.sz= self.sz[idx]
        self.Und= self.Und[idx]
        self.NSv= self.NSv[idx]
        self.DAge= self.DAge[idx]
        self.State= self.State[idx]

    def Use_TS_instead(self, TS_data, on_GNSS_Timestamps):

        if on_GNSS_Timestamps:

            time_data = np.array(TS_data.time)
            x_data = np.array(TS_data.x)
            y_data = np.array(TS_data.y)
            z_data = np.array(TS_data.z)

            # Überprüfung auf Duplikate und Entfernung
            unique_time_data, unique_indices = np.unique(time_data, return_index=True)
            unique_x_data = x_data[unique_indices]
            unique_y_data = y_data[unique_indices]
            unique_z_data = z_data[unique_indices]

            # print("Original time data:", len(TS_data.time))
            # print("Unique time data:", len(unique_time_data))
            # print("Original y data:", TS_data.y)
            # print("Unique y data:", unique_y_data)

            idx = np.where( ((self.time > unique_time_data[1]) & (self.time < unique_time_data[-1])) ) # Nicht optimale lösund da am Erste und letzte Position von GNSS aber zum testen okay

            interp_x = interp1d(unique_time_data, unique_x_data, kind='quadratic')
            self.x[idx] = interp_x(self.time[idx])
            interp_y = interp1d(unique_time_data, unique_y_data, kind='quadratic')
            self.y[idx] = interp_y(self.time[idx])
            interp_z = interp1d(unique_time_data, unique_z_data, kind='quadratic')
            self.z[idx] = interp_z(self.time[idx])
        else:
            self.time = np.array(TS_data.time)
            self.x = np.array(TS_data.x)
            self.y = np.array(TS_data.y)
            self.z = np.array(TS_data.z)

            self.sx= np.ones_like(np.array(TS_data.time)) * 0.005
            self.sy= np.ones_like(np.array(TS_data.time)) * 0.005
            self.sz= np.ones_like(np.array(TS_data.time)) * 0.005

            # Geoid Undulation, Number of Sat., DAge (Nur noch Platzhalter)
            self.Und= np.ones_like(np.array(TS_data.time)) 
            self.NSv= np.ones_like(np.array(TS_data.time))
            self.DAge= np.ones_like(np.array(TS_data.time))
            self.State= np.ones_like(np.array(TS_data.time)) * 7

    def writetofile( self, filename, type ):

        print("--------------------------------------------------------------------------------")
        print("Writing GNSS data to file")
        print(time.strftime("%H:%M:%S")," filename: ", filename)
        print(time.strftime("%H:%M:%S")," number: ", len(self.x))

        if (type == "gtsam"):

            data = np.c_[ self.time, self.x, self.y, self.z, self.sx, self.sy, self.sz, self.State ]

            np.savetxt(fname=filename, X=data, delimiter=" ", fmt="%10.5f")

        print("--------------------------------------------------------------------------------\n")

    # ================================================================================
    # Plot functions

    def plotXY( self, idx ):

        FZ = 16
            
        fig, ax = plt.subplots()
        plt.title('GNSS Position X / Y', fontsize=FZ)
        plt.plot(self.x, self.y, ".k", label = "GNSS fix")

        idx_out = np.where(self.State == 0)[0]
        plt.plot(self.x[idx_out], self.y[idx_out], ".r", label = "simulated outage")

        plt.plot(self.x[idx[-1]], self.y[idx[-1]], "*g", markersize = 24, label = "End initialization")
        plt.plot(self.x[0], self.y[0], ".g", markersize=12, label = "Start")
        plt.plot(self.x[-1], self.y[-1], ".r", markersize=12, label = "End" )

        plt.xlabel("X [m]", fontsize=FZ)
        plt.ylabel("Y [m]", fontsize=FZ)
        plt.grid("on")
        plt.legend()
        plt.axis("equal")

        ax.tick_params(axis='both', which='major', labelsize=FZ)

        return fig


    def plotZ( self, TS1data, TS2data ):

        FZ = 16
            
        fig, ax = plt.subplots()
        plt.title('GNSS Height Profile', fontsize=FZ)
        plt.plot(self.time - self.time[0], self.z, ".k", label="GNSSheight")

        if TS1data:
            plt.plot(TS1data.time, TS1data.z, ".g", label = "TS1 heights")
        if TS2data:
            plt.plot(TS2data.time, TS2data.z, ".r", label = "TS2 heights")     

        plt.ylabel("Z [m]", fontsize=FZ)
        plt.xlabel("time [s]", fontsize=FZ)
        plt.grid("on")
        plt.xlim((0, (self.time - self.time[0]).max()))
        plt.legend()

        ax.tick_params(axis='both', which='major', labelsize=FZ)
        
        return fig


    def plotSxSySz( self ):

        FZ = 16
            
        fig, (ax1, ax2, ax3) = plt.subplots(3)

        ax1.set_title('Standard Deviation GNSS Positions', fontsize=FZ)

        ax1.plot(self.time, self.sx*100, ".r")
        ax2.plot(self.time, self.sy*100, ".g")
        ax3.plot(self.time, self.sz*100, ".b")

        ax1.grid("on")
        ax2.grid("on")
        ax3.grid("on")

        ax1.set_ylabel("sx [cm]", fontsize=FZ)
        ax2.set_ylabel("sy [cm]", fontsize=FZ)
        ax3.set_ylabel("sz [cm]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)
        ax3.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlabel("time [s]", fontsize=FZ)
        ax2.set_xlabel("time [s]", fontsize=FZ)
        ax3.set_xlabel("time [s]", fontsize=FZ)

        ax1.set_xlim([self.time[0], self.time[-1]])
        ax2.set_xlim([self.time[0], self.time[-1]])
        ax3.set_xlim([self.time[0], self.time[-1]])

        return fig

    def plotNumberOfSattelites( self ):

        FZ = 16

        if len(self.NSv) > len(self.time):
            print("number of NSv msg to large")

            d = len(self.NSv) - len(self.time)
            self.NSv = self.NSv[0:-(d)]


        if len(self.time) > len(self.NSv):
            print("number of NSv msg to less")

        fig, ax = plt.subplots()
        plt.title('Number Of Sattelites', fontsize=FZ)
        plt.plot(self.time, self.NSv, ".k")

        plt.ylabel("Number Of Satellites", fontsize=FZ)
        plt.xlabel("time [s]", fontsize=FZ)
        plt.grid("on")
        plt.xlim((self.time[0], self.time[-1]))

        ax.tick_params(axis='both', which='major', labelsize=FZ)


        ax.set_xlabel("time [s]", fontsize=FZ)

        return fig
        
    def plotUndulation( self ):

        FZ = 16

        if len(self.Und) > len(self.time):
            print("number of Und msg to large")

            d = len(self.Und) - len(self.time)
            self.Und = self.Und[0:-(d)]

        if len(self.time) > len(self.Und):
            print("number of NSv msg to less")

        fig, ax = plt.subplots()
        plt.title('Geoid Undulation', fontsize=FZ)
        plt.plot(self.time, self.Und, ".k")

        plt.ylabel("Geoid Undulation [m]", fontsize=FZ)
        plt.xlabel("time [s]", fontsize=FZ)
        plt.grid("on")
        plt.xlim((self.time[0], self.time[-1]))

        ax.tick_params(axis='both', which='major', labelsize=FZ)

        ax.set_xlabel("time [s]", fontsize=FZ)

        return fig

    def plotGNSSstate( self, system = "Phenobot" ):

        FZ = 16

        if len(self.State) > len(self.time):
            print("number of State msg to large")

            d = len(self.State) - len(self.time)
            self.State = self.State[0:-(d)]

        if len(self.time) > len(self.State):
            print("number of State msg to less")

        fig, ax = plt.subplots()
        plt.title('GNSS State', fontsize=FZ)

        if system == "Phenobot":
            plt.plot(self.time[0], 100, color="black", label="0:  No solution computed")
            plt.plot(self.time[0], 101, color="black", label="1:  Unknown solution")
            plt.plot(self.time[0], 102, color="black", label="2:  Single point position")
            plt.plot(self.time[0], 103, color="black", label="3:  Standard Pseudorange Differential Solution (DGPS)")
            plt.plot(self.time[0], 104, color="black", label="4:  SBAS satellite used for differential corrections.")
            plt.plot(self.time[0], 105, color="black", label="5:  Omnistar VBS Position (L1 sub-meter)")
            plt.plot(self.time[0], 106, color="black", label="6:  Floating RTK ambiguity solution")
            plt.plot(self.time[0], 107, color="black", label="7:  Integer RTK ambiguity solution")
            plt.plot(self.time[0], 108, color="black", label="8:  PPP with float ambiguities")
            plt.plot(self.time[0], 109, color="black", label="9:  PPP with integer ambiguities")
            plt.plot(self.time[0], 110, color="black", label="10: Fixed location solution position")
        else:
            plt.plot(self.time[0], 110, color="black", label="3: Integer RTK ambiguity solution")

        plt.legend(fontsize=12, loc= "upper right")
        plt.plot(self.time, self.State, ".k")
        plt.ylabel("GNSS State [-]", fontsize=FZ)
        plt.xlabel("time [s]", fontsize=FZ)
        plt.grid("on")
        plt.xlim((self.time[0], self.time[-1]))

        ax.tick_params(axis='both', which='major', labelsize=FZ)

        ax.set_xlabel("time [s]", fontsize=FZ)

        ax.set_ylim((0,15))

        return fig


# ========================================================================================
# SBG GNSS heading and pitch data
    
class GNSS_headingpitch:
    def __init__(self, time = 0.0, time_frame ="undefined",
                        heading = np.empty( (), dtype= float ), pitch = np.empty( (), dtype= float ), hp_frame = "undefined",
                        sheading = np.empty( (), dtype= float ), spitch = np.empty( (), dtype= float ), state=np.empty( (), dtype= float ), baseline=np.empty( (), dtype= float )):

        self.time=time
        self.time_frame = time_frame
            
        self.heading=heading
        self.pitch=pitch
        self.hp_frame=hp_frame

        self.sheading=sheading
        self.spitch=spitch
        self.state=state

        self.baseline=baseline
    
    def writetofile(self, filename):

        print("--------------------------------------------------------------------------------")
        print("Writing GNSS heading and pitch data to file")
        print(time.strftime("%H:%M:%S")," filename: ", filename)
        print(time.strftime("%H:%M:%S")," number: ", len(self.heading))

        data = np.c_[ self.time, self.heading, self.pitch, self.sheading, self.spitch, self.state ]

        np.savetxt(fname=filename, X=data, delimiter=" ", fmt="%10.10f")

        print("--------------------------------------------------------------------------------\n")


    # def convert_from_NED_to_ENU(self):

    #     #R_NED_to_ENU = geobase.RotmatZ( geobase.deg2rad( -90 ) ) @ geobase.RotmatY( geobase.deg2rad( 180 ) )

    #     R_NED_to_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
            
    #     for i in np.arange( 0, len(self.heading) ):

    #         R_BN_z = R_NED_to_ENU @ geobase.RotmatZ( self.heading[i] )
    #         R_BN_y = R_NED_to_ENU @ geobase.RotmatZ( self.pitch[i] )

    #         rpy_z = geobase.Rotmat2Euler( R_BN_z )
    #         rpy_y = geobase.Rotmat2Euler( R_BN_y )

    #         self.heading[i] = rpy_z[2]
    #         self.pitch[i] = rpy_y[2]

    def toNED(self):
        if self.hp_frame == "ENU":

            print("[INFO] Orientation Angle Transformation: From ENU to NED Frame")
            R_ENU_NED = geobase.RotmatY( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( 90 ) )
                    
            for i in np.arange(0, len(self.heading)):

                R_BN = R_ENU_NED @ geobase.RotmatZ(self.heading[i]) @ geobase.RotmatY(self.pitch[i])
                rpy = geobase.Rotmat2Euler( R_BN )

                self.heading[i] = rpy[2]
                self.pitch[i] = rpy[1]

            self.hp_frame = "NED"
    
    def toENU(self):
        if self.hp_frame == "NED":

            print("[INFO] Orientation Angle Transformation: From NED to ENU Frame")
            R_NED_ENU = geobase.RotmatZ( geobase.deg2rad( -90 ) ) @ geobase.RotmatY( geobase.deg2rad( 180 ) )
                    
            for i in np.arange(0, len(self.heading)):

                R_BN = R_NED_ENU @ geobase.RotmatZ(self.heading[i]) @ geobase.RotmatY(self.pitch[i])
                rpy = geobase.Rotmat2Euler( R_BN )

                self.heading[i] = rpy[2]
                self.pitch[i] = rpy[1]

            self.hp_frame = "ENU"

    def cut_by_Index(self,idx_hp):
        self.time=self.time[idx_hp]
        self.heading=self.heading[idx_hp]
        self.pitch=self.pitch[idx_hp]
        self.sheading=self.sheading[idx_hp]
        self.spitch=self.spitch[idx_hp]
        self.state=self.state[idx_hp]
        self.baseline=self.baseline[idx_hp]

    def Use_TS_instead(self, Time, Heading_TS, Pitch_TS, on_GNSS_Timestamps):

        if on_GNSS_Timestamps:
            time_data = np.array(Time)
            Heading = np.array(Heading_TS)
            Pitch = np.array(Pitch_TS)

            # Überprüfung auf Duplikate und Entfernung
            unique_time_data, unique_indices = np.unique(time_data, return_index=True)
            unique_Heading = Heading[unique_indices]
            unique_Pitch = Pitch[unique_indices]

            # print("Original time data:", len(TS_data.time))
            # print("Unique time data:", len(unique_time_data))
            # print("Original y data:", TS_data.y)
            # print("Unique y data:", unique_y_data)

            idx = np.where( ((self.time > unique_time_data[1]) & (self.time < unique_time_data[-1])) ) # Nicht optimale lösund da am Erste und letzte Position von GNSS aber zum testen okay

            interp_Head = interp1d(unique_time_data, unique_Heading, kind='quadratic')
            diff = self.heading[idx]-interp_Head(self.time[idx])
            plt.figure()
            plt.plot(self.time[idx],geobase.rad2deg(diff))
            plt.grid()
            plt.show()

            self.heading[idx] = interp_Head(self.time[idx])
            interp_Pitch = interp1d(unique_time_data, unique_Pitch, kind='quadratic')
            self.pitch[idx] = interp_Pitch(self.time[idx])

        else:
            self.time = Time
            self.heading = Heading_TS
            self.pitch = Pitch_TS

            self.sheading = np.ones_like(Time) * geobase.deg2rad(0.04)
            self.spitch = np.ones_like(Time) * geobase.deg2rad(0.04)
            self.state = np.zeros_like(Time)
            self.baseline=np.zeros_like(Time)

    def plotHP(self):

        FZ = 16
            
        fig, ax = plt.subplots()
        plt.title('GNSS Heading', fontsize=FZ)

        plt.plot(self.time, geobase.rad2deg( self.heading ), ".k", markersize=4)
        #plt.plot(self.time, geobase.rad2deg( self.pitch ), ".r")

        plt.ylabel("Heading [°]", fontsize=FZ)
        plt.xlabel("time [s]", fontsize=FZ)
        plt.grid("on")
        plt.xlim((self.time[0], self.time[-1]))
        plt.legend(["heading"])

        ax.tick_params(axis='both', which='major', labelsize=FZ)
        
        return fig


    def plot_sig( self ):

        FZ = 16
            
        fig, (ax1, ax2) = plt.subplots(2)

        ax1.set_title('Standard Deviation GNSS Positions', fontsize=FZ)

        ax1.plot(self.time, geobase.rad2deg( self.sheading ), ".r")
        ax2.plot(self.time, geobase.rad2deg( self.spitch) , ".g" )

        ax1.grid("on")
        ax2.grid("on")

        ax1.set_ylabel("s_head [deg]", fontsize=FZ)
        ax2.set_ylabel("s_pitch [deg]", fontsize=FZ)

        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax2.tick_params(axis='both', which='major', labelsize=FZ)

        ax1.set_xlabel("time [s]", fontsize=FZ)
        ax2.set_xlabel("time [s]", fontsize=FZ)

        ax1.set_xlim([self.time[0], self.time[-1]])
        ax2.set_xlim([self.time[0], self.time[-1]])

        return fig
    
    def plot_baseline( self ):
        
        
        FZ = 16
            
        fig, (ax1) = plt.subplots(1)

        ax1.set_title('GNSS Baseline', fontsize=FZ)

        ax1.plot(self.time, self.baseline , ".k")

        ax1.grid("on")
        ax1.set_ylabel("baseline [m]", fontsize=FZ)
        ax1.tick_params(axis='both', which='major', labelsize=FZ)
        ax1.set_xlabel("time [s]", fontsize=FZ)
        ax1.set_xlim([self.time[0], self.time[-1]])

        return fig
    



class TS_data:

    def __init__(self,
                 time = [],
                 x = [],
                 y = [],
                 z = [], state = []):

        self.time=time
        self.x = x
        self.y = y
        self.z = z
        self.state = state

    def read_from_ascii(self, path, fileformat):

        # find all filenames with ending traj
        files = glob.glob(path + "*.traj")

        print("files= ", files)

        if files:

            if fileformat == "normal":
                ts_data = np.loadtxt( files[0], delimiter=',', comments="#")
                self.time = ts_data[:,0]
                self.x = ts_data[:,1]
                self.y = ts_data[:,2]
                self.z = ts_data[:,3]

            if fileformat == "correction":
                ts_data = np.loadtxt( files[0], delimiter=',', comments="#")
                self.time = ts_data[:,0]
                self.x = ts_data[:,1] 
                self.y = ts_data[:,2]
                self.z = ts_data[:,3] +0.332 # Geoidundulation 47.332 but in rts tools only +47 # From NHN in GES80 Hight + Pfeilerhöhe    

            if fileformat == "correctionPlus2":
                # with estimated correction of TS-Origin-Point (MS60 22.07.24) 
                ts_data = np.loadtxt( files[0], delimiter=',', comments="#")
                self.time = ts_data[:,0]+ 0.006 
                self.x = ts_data[:,1] +0.001
                self.y = ts_data[:,2] -0.0005
                self.z = ts_data[:,3] -0.0026+0.332 # Geoidundulation 47.332 but in rts tools only +47 # From NHN in GES80 Hight + Pfeilerhöhe 

            # convert time stamp from UNIX to seconds of week
            self.time = (( (self.time / 86400) + 4 ) - (np.floor(self.time / 86400) + 4 ) ) * 86400
                
            # set ts state
            self.state = np.ones( self.time.shape )

            P = (
                "\n ____________________________________________________________\n"
                f"| TS Data Reading Report                                       |\n"
                f"| - filename {files}                                          \n"
                f"| - First position xyz:   {self.x[0]:.3f} {self.y[0]:.3f} {self.z[0]:.3f} [m] \n"
                f"| - Number of positions   {len(self.time):.0f} [m] \n"
                f"| - Mean rate   {1/np.mean(np.diff(self.time)):.0f} [Hz] \n"
                "|______________________________________________________________|\n"
            )

            print(P)

        else:
            self.time = []
            self.x = []
            self.y = []
            self.z = []
            self.state = []

            P = (
                "\n ____________________________________________________________\n"
                f"| No TS Data Found                                            |\n"
                 "|_____________________________________________________________|\n"
            )

            print(P)

    def interpolate_to_2nd_TS(self, TS_data):
        
        # only timeinterval with both TS 
        starttime = max(TS_data.time[0],self.time[0]) # start time 
        endtime = min(TS_data.time[-1],self.time[-1]) # end time 

        id_02 = np.where( ((TS_data.time > starttime+1) & (TS_data.time < endtime-1)) )
        id_02 = id_02[0]
        id_01 = np.where( ((self.time > starttime) & (self.time < endtime)) )
        id_01 = id_01[0]

        self.time=self.time[id_01]
        self.x=self.x[id_01]
        self.y=self.y[id_01]
        self.z=self.z[id_01]
        TS_data.time=TS_data.time[id_02]
        TS_data.x=TS_data.x[id_02]
        TS_data.y=TS_data.y[id_02]
        TS_data.z=TS_data.z[id_02]

        interp_x = interp1d(self.time, self.x, kind='linear')
        self.x = interp_x(TS_data.time)
        interp_y = interp1d(self.time, self.y, kind='linear')
        self.y = interp_y(TS_data.time)
        interp_z = interp1d(self.time, self.z, kind='linear')
        self.z = interp_z(TS_data.time)
        self.time = TS_data.time

        # set ts state
        self.state = np.ones( self.time.shape )
        TS_data.state = np.ones( TS_data.time.shape )

        return(TS_data)
    
    def orientation_from_2TS(self,TS_data2):
        Heading_TS = np.zeros(len(TS_data2.time))
        Pitch_TS = np.zeros(len(TS_data2.time))
        Base_TS = np.zeros((len(TS_data2.time),3))

        for i in range(0, len(TS_data2.time)):
            basevector= np.array([self.x[i],self.y[i],self.z[i]])-np.array([TS_data2.x[i],TS_data2.y[i],TS_data2.z[i]])
            # if Heading is estimated in UTM coordinates
            #[lat, lon] = utm2ell(ts_data_02.y[i],ts_data_02.x[i],32) 
            #Meridiankovergenz = (lon-geobase.deg2rad(9))*np.sin(lat)

            Base_vec_norm = basevector/np.linalg.norm(basevector)
            Base_TS[i,:]=basevector

            Heading_TS[i] = np.arctan2(Base_vec_norm[1], Base_vec_norm[0]) # if Heading is estimated in UTM coordinates: -Meridiankovergenz
            Pitch_TS[i] = np.arctan2(np.linalg.norm(basevector[0:2]), basevector[2])- geobase.deg2rad(90) 
        return Heading_TS, Pitch_TS, Base_TS
    
    def outliertest_by_baselength(self,TS_Data):
        if len(self.x) != len(TS_Data.x):
            print('ERROR: TS Data has not the same size. Please interpolate before!')
            print("-------------------------------------------------------------------------------\n") 
        else:
            baselength = np.linalg.norm(np.array([self.x,self.y,self.z])-np.array([TS_Data.x,TS_Data.y,TS_Data.z]), axis=0)
            no_outlier = np.full(len(baselength), True)
            for i in range(len(baselength)):
                if  baselength[i] > 0.947 or baselength[i] < 0.935: # calibration measurment results
                    no_outlier[i] = False

            
            self.time = self.time[no_outlier]
            self.x = self.x[no_outlier]
            self.y = self.y[no_outlier]
            self.z = self.z[no_outlier]
            print('Outlier detection: Reject '+ str(len(TS_Data.x)-len(self.x))+ ' from '+ str(len(TS_Data.x))+' data')
            TS_Data.time = TS_Data.time[no_outlier]
            TS_Data.x = TS_Data.x[no_outlier]
            TS_Data.y = TS_Data.y[no_outlier]
            TS_Data.z = TS_Data.z[no_outlier]
            print("-------------------------------------------------------------------------------\n") 

            # set ts state
            self.state = np.ones( self.time.shape )
            TS_Data.state = np.ones( TS_Data.time.shape )

            return TS_Data #,baselength[no_outlier]
        
    def intersect_with_IMU(self, IMU):
        # ===============================================================       
        # Cut data concerning IMU time

        idx = np.argwhere( (self.time > IMU.time[0]) & (self.time < IMU.time[-1]) ).flatten()

        self.time = self.time[idx]
        self.x = self.x[idx]
        self.y = self.y[idx]
        self.z = self.z[idx]

        # ===============================================================
        # Find minimum time differences indices to IMU data

        t_min = np.zeros( (self.x.shape) )

        for i in np.arange(0,len(self.x)):
            delta = np.abs( IMU.time - self.time[i] )
            idx = np.argmin(delta)
            t_min[i] = IMU.time[idx]

        # ===============================================================    
        # Rewrite new TS data
        self.time = t_min

        # set ts state
        self.state = np.ones( self.time.shape )

    def cut_by_time( self, t_start, t_end ):
        idx = np.argwhere( (self.time > t_start) & (self.time < t_end) ).flatten()

        self.time = self.time[idx]
        
        self.x = self.x[idx]
        self.y = self.y[idx]
        self.z = self.z[idx]
        self.state = self.state[idx]

    def cut_by_Index(self,idx):
        self.time=self.time[idx]
        self.x=self.x[idx]
        self.y=self.y[idx]
        self.z=self.z[idx]
        self.state = self.state[idx]


    def intersect( self, time_vec, method = "nearest_neigbor" ):

        if method == "nearest_neigbor":
            t_min = np.zeros( (self.x.shape) )

            for i in np.arange(0,len(self.x)):
            
                delta = np.abs( time_vec - self.time[i] )

                idx = np.argmin(delta)

                t_min[i] = time_vec[idx]


            self.time = t_min
            self.state = np.ones( self.time.shape )


    def plot_xy(self, gnss = [] ):
        
        FZ = 16

        fig = plt.figure()

        plt.plot(self.x, self.y, ".b", markersize=4 )

        plt.grid("on")
        plt.xlabel("UTM East", fontsize=FZ )
        plt.ylabel("UTM North", fontsize=FZ )
        plt.legend(["TS data"])
        plt.axis("equal")
                
        return fig


    def writetofile(self, type, filename):

        if (type == "gtsam"):
            data = np.c_[ self.time, self.x, self.y, self.z, self.state ]

            np.savetxt(fname=filename, X=data, delimiter=" ", fmt="%10.5f")

            print("- [INFO] wrote TS data to .traj file")
            print("")





