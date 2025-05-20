from re import I
import numpy as np

from srcpython.geodetictools import _01base as geobase
from srcpython.geodetictools import _03CoordinateTransformations as geotransform
from srcpython.geodetictools._04Visualization import *
import matplotlib.pyplot as plt

# Class import
from srcpython.geodetictools._01base_classes import Trajectory3D, IMUdata, GNSSdata, GNSS_headingpitch, rotation, translation, velocity


import time


class SBGdata:
    def __init__(self):

        self.trajectory = Trajectory3D()
        self.IMUdata = IMUdata()
        self.GNSSdata = GNSSdata()
        self.GNSS_headingpitch = GNSS_headingpitch()

        self.time_max = 5
        self.skip_time = 3 # sec

        self.imu_start = self.skip_time * 100 # time * rate
        self.gps_start = self.skip_time * 5 # time * rate
        self.ekf_start = self.skip_time * 10
        self.hdt_start = self.skip_time * 5 # gnss heading
        
    def readSBG_EKF(self, path: str = "", xyz_frame: str = "", rpy_frame: str = "", offset: np.ndarray = np.array([0,0,0])):

            filenames = np.array( ("EkfEuler.bin","EkfNav.bin") )

            TimeEuler=[]
            TimeEkf=[]

            time_h = 5 # assumption: no measurements in the first 5 hours of a day
            time_max = 3600 * 24 * 7 # maximal possible time stamp

            # loop over all fiels in directory
            for x in range(0,len(filenames)):

                # current file name
                fullfile = path + '/' + filenames[x]

                # 1) Ekf Euler
                if filenames[x] == "EkfEuler.bin":
                    with open(fullfile, 'rb') as fid:

                        data_array2 = np.fromfile(fid, np.double).reshape((-1, 7))
                        
                        # Delete Wrong measurements
                        #idx_w = np.where( (data_array2[:,0] >  self.time_max * 3600 ) & (data_array2[:,0] < time_max) )

                        idx_w = np.arange( self.ekf_start, len(data_array2[:,0]) )

                        rpy = np.c_[data_array2[idx_w,1], data_array2[idx_w,2], data_array2[idx_w,3]]

                        TimeEuler = data_array2[idx_w,0]

                        print("- [INFO] read EKF euler angles")

                # 2) Ekf EkfNav
                elif filenames[x] == "EkfNav.bin":
                    with open(fullfile, 'rb') as fid:
                        SbgEkfNav = np.fromfile(fid, np.double).reshape((-1, 14))

                        #idx_w = np.where( (SbgEkfNav[:,0] >  self.time_max * 3600 ) & (SbgEkfNav[:,0] < time_max) )
                        #idx_w = idx_w[0]

                        idx_w = np.arange( self.ekf_start, len(SbgEkfNav[:,0]) )

                        # Time Vector
                        TimeEkf = SbgEkfNav[idx_w,0]

                        # Choose Right Coordinate Frame
                        # self.xyz_frame -- Zielsystem
                        # xyz_frame --> Frame of SBG

                        # to UTM transformation
                        if ((xyz_frame == "LLH") & (self.trajectory.xyz_frame == "ECEF")):

                            # states for statesall
                            lat = geobase.deg2rad( SbgEkfNav[idx_w,1] )
                            lon = geobase.deg2rad( SbgEkfNav[idx_w,2] )
                            H = SbgEkfNav[idx_w,3]

                            # velocity
                            vN = SbgEkfNav[idx_w,8]
                            vE = SbgEkfNav[idx_w,9]
                            vD = SbgEkfNav[idx_w,10]

                            x, y, z = geotransform.ell2xyz( lat, lon, H )

                            xyz = np.c_[x,y,z]
                            V = np.c_[vN,vE,vD]

                            xyz_sig = np.c_[SbgEkfNav[idx_w,4], SbgEkfNav[idx_w,5], SbgEkfNav[idx_w,6]]

                            vxyz_sig = np.c_[ SbgEkfNav[idx_w,11], SbgEkfNav[idx_w,12], SbgEkfNav[idx_w,13] ]


                        elif ((xyz_frame == "LLH") & (self.trajectory.xyz_frame == "UTM")):
                            
                            # states for statesall
                            lat = geobase.deg2rad( SbgEkfNav[idx_w,1] )
                            lon = geobase.deg2rad( SbgEkfNav[idx_w,2] )
                            H = SbgEkfNav[idx_w,3] + SbgEkfNav[idx_w,7] # Ellip. height = Geoid Height + Undulation

                            # convert to UTM coordinates
                            N,E = geotransform.ell2utm(lat,lon,Zone=32)

                            # velocities
                            vN = SbgEkfNav[idx_w,8]
                            vE = SbgEkfNav[idx_w,9]
                            vD = SbgEkfNav[idx_w,10]

                            xyz = np.c_[E,N,H]

                            # apply global offset
                            xyz[:,0] -= offset[0]
                            xyz[:,1] -= offset[1]
                            xyz[:,2] -= offset[2]

                            V = np.c_[vN,vE,vD]

                            xyz_sig = np.c_[ SbgEkfNav[idx_w,4], SbgEkfNav[idx_w,5], SbgEkfNav[idx_w,6] ]

                            vxyz_sig = np.c_[ SbgEkfNav[idx_w,11], SbgEkfNav[idx_w,12], SbgEkfNav[idx_w,13] ]

                        else:
                            print("System not defined")
                        
                        #print("- [INFO] read EKF positions")
                        
            # Find outlier timestamps
            
            # Timestamps

            # Find intersecting interval of EKF and Euler Solution
            xy, x_ind, y_ind = np.intersect1d( TimeEkf, TimeEuler, return_indices=True )

            # fill rotation and ...
            self.trajectory.numberofstates = len(x_ind)

            # intersecting time
            self.trajectory.time = TimeEkf[x_ind]

            self.trajectory.rotation = [rotation() ] * self.trajectory.numberofstates
            self.trajectory.translation = [translation() ] * self.trajectory.numberofstates
            self.trajectory.velocity = [velocity() ] * self.trajectory.numberofstates
            
            # Translation
            for i in range( 0, self.trajectory.numberofstates ):
                self.trajectory.translation[i] = translation(time=TimeEkf[x_ind[i]],
                                        frame="ECEF",
                                        xyz= geobase.vector3( xyz[x_ind[i],0], xyz[x_ind[i],1], xyz[x_ind[i],2] ),
                                        cov = geobase.matrix33(np.power(xyz_sig[x_ind[i],0],2),0,0,0,np.power(xyz_sig[x_ind[i],1],2),0,0,0,np.power(xyz[x_ind[i],2],2)))
            
                # Velocity
                self.trajectory.velocity[i] = velocity(time=TimeEkf[i],
                                                    frame="NED",
                                                    V=geobase.vector3(vN[i],vE[i],vD[i]),
                                                    cov = geobase.matrix33(np.power(vxyz_sig[x_ind[i],0],2),0,0,0,np.power(vxyz_sig[x_ind[i],1],2),0,0,0,np.power(vxyz_sig[x_ind[i],2],2)))

            # Rotation
            for i in range( 0, self.trajectory.numberofstates ):
                            self.trajectory.rotation[i] = rotation(time=TimeEuler[y_ind[i]],
                                                                    frame=rpy_frame,
                                                                    rpy=geobase.vector3(rpy[y_ind[i],0],rpy[y_ind[i],1],rpy[y_ind[i],2]),
                                                                    cov=geobase.matrix33(0,0,0,0,0,0,0,0,0))


            self.trajectory.statesall = np.c_[ self.trajectory.time, xyz[x_ind,:], V[x_ind,:], rpy[y_ind,:],
                                               np.zeros((self.trajectory.numberofstates,3)), np.zeros((self.trajectory.numberofstates,3))]
            
            #print("- [INFO] first state", self.trajectory.statesall[0,:])
            #print("- [INFO] last position UTM", self.trajectory.statesall[-1,1], self.trajectory.statesall[-1,2])

            # ----------------------------------------------------------------------------------------------------
            # INTERSECTION OF SBG Topics
            # ----------------------------------------------------------------------------------------------------     

            # ----------------------------------------------------------------------------------------------------
            # DELTE WRONG TIME STAMPS
            # ----------------------------------------------------------------------------------------------------
            # Delete Wrong measurements
            #time_h = 5 # assumption: no measurements in the first 5 hours of a day
            #time_max = 3600 * 24 * 7
            #idx_w = np.where( (self.trajectory.time >  self.time_max * 3600 ) & (self.trajectory.time < time_max) )
            #idx_w = idx_w[0]

            idx_w = np.arange( 0, len(self.trajectory.time) )

            # Delete States
            self.trajectory.time = [ self.trajectory.time[i] for i in idx_w ]
            self.trajectory.numberofstates = len(self.trajectory.time)
            self.trajectory.rotation = [self.trajectory.rotation[i] for i in idx_w ]
            self.trajectory.translation = [self.trajectory.translation[i] for i in idx_w ]
            self.trajectory.statesall = self.trajectory.statesall[idx_w,:]

            # Bias states
            self.trajectory.biasstatesall = []

            # TODO: states left
            # - velocity

            print("-------------------------------------------------------------------------------")
            print("SBG Trajectory Reading Report")
            print( time.strftime("%H:%M:%S"), " first state trajectory : " , self.trajectory.statesall[0,:])
            print( time.strftime("%H:%M:%S"), " number of states: " , len(self.trajectory.statesall[0,:]))
            print( time.strftime("%H:%M:%S"), " time interval: " , self.trajectory.time[-1] - self.trajectory.time[0], " [s], [", self.trajectory.time[0], self.trajectory.time[-1], "]")
            print( time.strftime("%H:%M:%S"), " roll pitch yaw frame: ", rpy_frame)
            print( time.strftime("%H:%M:%S"), " xyz frame: ", xyz_frame )
            print( time.strftime("%H:%M:%S"), " bias states: ", self.trajectory.biasstatesall )
            print("-------------------------------------------------------------------------------\n")

    def readIMUdata(self, path, time_frame ,acc_frame, gyro_frame):

        print("--------------------------------------------------------------------------------")
        print("[INFO] SBG IMU data") 

        filename = "Imu.bin"
        fullfile = path  + filename

        # read data from folder
        with open(fullfile, 'rb') as fid:
            data_array = np.fromfile(fid, np.double).reshape((-1, 7))
        
        # Time Frame
        if time_frame == "UTC":
            self.IMUdata.time = data_array[:,0] + 18
            self.IMUdata.time_frame = "UTC"
            print("- [INFO] time frame: ", self.IMUdata.time_frame)
            print("- [INFO] time vector: ",self.IMUdata.time)

        elif time_frame == "GPST":
            self.IMUdata.time = data_array[:,0]
            self.IMUdata.time_frame = time_frame
            print("- [INFO] time frame: ", self.IMUdata.time_frame) 
        else:
            print("- [INFO] Error! unknown time frame")

        # delete wrong time stamps at the beginning
        #time_h = 5 # assumption: no measurements in the first 5 hours of a day
        #time_big = 7 * 24 # outlier time stamps

        #idx_w = np.where( (data_array[:,0] >  self.time_max * 3600) & (data_array[:,0] <  time_big * 3600) )
        #idx_w = idx_w[0]

        # Sort data array with time stamps
        sorted_indices = np.argsort(data_array[:, 0])

        # Matrix basierend auf den sortierten Indizes umordnen
        data_array = data_array[sorted_indices]


        idx_w = np.arange( self.imu_start, len(data_array[:,0]) )


        # IMU time
        self.IMUdata.time=data_array[idx_w,0]

        # Acceleration
        self.IMUdata.accx=data_array[idx_w,1]
        self.IMUdata.accy=data_array[idx_w,2]
        self.IMUdata.accz=data_array[idx_w,3]
        self.IMUdata.acc_frame = acc_frame

        # Gyroscope data
        self.IMUdata.gyrox=data_array[idx_w,4]
        self.IMUdata.gyroy=data_array[idx_w,5]
        self.IMUdata.gyroz=data_array[idx_w,6]
        self.IMUdata.gyro_frame = gyro_frame
        
        
        print("- lenght of IMU data",  self.IMUdata.gyrox.shape )
        print("- [INFO] ... done")
        print("--------------------------------------------------------------------------------")


    def readGNSSPosdata( self, path: str = "", time_frame: str = "", offset: np.ndarray = np.array([0,0,0]) ):

        filename = "GnssPos.bin"
        fullfile = path + filename

        # read data from folder
        with open(fullfile, 'rb') as fid:
            data_array = np.fromfile(fid, np.double).reshape((-1, 11))

        # delete wrong time stamps at the beginning
        #time_h = 5 # assumption: no measurements in the first 5 hours of a day
        #idx_w = np.where( data_array[:,0] >  self.time_max * 3600 )

        idx_w = np.arange( self.gps_start, len(data_array[:,0]) )

        # Time
        print("- [INFO] gps time vector: ", data_array[:,0])

        self.GNSSdata.time=data_array[idx_w,0]
        self.GNSSdata.time_frame = time_frame

        lon = geobase.deg2rad( data_array[idx_w,2] )
        lat = geobase.deg2rad( data_array[idx_w,1] )

        # ellip. height = height(SBG) + undulation
        alt = data_array[idx_w,3] + data_array[idx_w,7]
        self.GNSSdata.y, self.GNSSdata.x = geotransform.ell2utm(lat,lon,Zone=32)

        self.GNSSdata.x -= offset[0]
        self.GNSSdata.y -= offset[1]
        self.GNSSdata.z = alt - offset[2]

        # UTM std
        self.GNSSdata.sx=data_array[idx_w,4]
        self.GNSSdata.sy=data_array[idx_w,5]
        self.GNSSdata.sz=data_array[idx_w,6]

        # Geoid Undulation, Number of Sat., DAge
        self.GNSSdata.Und = data_array[idx_w,7]
        self.GNSSdata.NSv =data_array[idx_w,8]
        self.GNSSdata.DAge=data_array[idx_w,9]

        self.GNSSdata.State = self.determineGNSSstate( data_array[idx_w,10] )

        print("-------------------------------------------------------------------------------")
        print("SBG GNSS data Reading Report")
        print( time.strftime("%H:%M:%S"), " first position: " , self.GNSSdata.x, self.GNSSdata.y, self.GNSSdata.z)
        print( time.strftime("%H:%M:%S"), " number of states: " , len(self.GNSSdata.x))
        print( time.strftime("%H:%M:%S"), " time interval: " , self.GNSSdata.time[-1] - self.GNSSdata.time[0], " [s], [", self.GNSSdata.time[0], self.GNSSdata.time[-1], "]")
        print( time.strftime("%H:%M:%S"), " xyz frame: ",  self.GNSSdata.xyz_frame )
        print("-------------------------------------------------------------------------------\n")


        # convert state from dec 2 binary, then bits 6 - 11 are solution status
        #Gnss_satus = dec2bin(self.GNSSdata.State);
        #Gnss_sol = bin2dec(Gnss_satus(:, end-11:end-6));


    def determineGNSSstate(self, int32_vec):
        
        gnss_state_int23 = np.array( int32_vec, dtype="uint32")
        gnss_state = np.zeros( gnss_state_int23.shape, dtype="int")

        for i in range( 0, len(gnss_state_int23) ):
            
            # GNSS pos status of the ith epoch as binary number
            binary = np.binary_repr( gnss_state_int23[i], width=32 )

            # GNSS pos staus as integer    
            gnss_state[i] = int(binary[20:26], 2)

        return gnss_state



    def readGNSS_heading_pitch(self, path, time_frame, hp_frame ):
        
        print("--------------------------------------------------------------------------------")
        print("[INFO] SBG GNSS heading & pitch data")

        filename = "GnssHdt.bin"
        fullfile = path + '/' + filename

        # read data from folder
        with open(fullfile, 'rb') as fid:
            data_array = np.fromfile(fid, np.double).reshape((-1, 6))

        # delete wrong time stamps at the beginning
        #time_h = 5 # assumption: no measurements in the first 5 hours of a day
        #time_max = 3600 * 24 * 7
        #idx_w = np.where( (data_array[:,0] >  self.time_max * 3600) & (data_array[:,0] < time_max) )

        idx_w = np.arange( self.gps_start, len(data_array[:,0]) )

    
        self.GNSS_headingpitch.time=data_array[idx_w,0]
        self.GNSS_headingpitch.time_frame = time_frame

        print("Time=", self.GNSS_headingpitch.time)
        
        self.GNSS_headingpitch.heading=geobase.deg2rad( data_array[idx_w,1] )
        self.GNSS_headingpitch.pitch=geobase.deg2rad( data_array[idx_w,2] )

        print("Heading angle: ", data_array[:,1] )

        print( geobase.rad2deg( self.GNSS_headingpitch.heading) )
        print( geobase.rad2deg( self.GNSS_headingpitch.pitch) )

        self.GNSS_headingpitch.hp_frame=hp_frame

        print(self.GNSS_headingpitch.pitch)
        print(self.GNSS_headingpitch.heading)

        self.GNSS_headingpitch.sheading=geobase.deg2rad( data_array[idx_w,3] )
        self.GNSS_headingpitch.spitch= geobase.deg2rad( data_array[idx_w,4] )
        self.GNSS_headingpitch.state=data_array[idx_w,5]
        self.GNSS_headingpitch.baseline=np.zeros(data_array[idx_w,5].shape)

        print("[INFO] state vector: ", self.GNSS_headingpitch.state)
        print("[INFO] state vector min max: ", np.min( self.GNSS_headingpitch.state), " ", np.max( self.GNSS_headingpitch.state) )

    def fill_IMUdata_Gaps(self):
        # Fill gaps by linear interpolation
        differenzen = []
        new_IMU_data_time = []
        new_accx = []
        new_accy = []
        new_accz = []
        new_gyrox = []
        new_gyroy = []
        new_gyroz = []
        for i in range(1, len(self.IMUdata.time)):
            differenz = self.IMUdata.time[i] - self.IMUdata.time[i - 1]
            if differenz > 0.011:
                if differenz > 0.5:
                    print('WARNING: BIG GAP IN SBG DATA!!!')
                for j in range(0, int(round(differenz/0.01))):
                    new_IMU_data_time.append(self.IMUdata.time[i - 1]+0.01*j)
                    new_accx.append(self.IMUdata.accx[i - 1]+(self.IMUdata.accx[i]-self.IMUdata.accx[i - 1])*j/int(differenz/0.01))
                    new_accy.append(self.IMUdata.accy[i - 1]+(self.IMUdata.accy[i]-self.IMUdata.accy[i - 1])*j/int(differenz/0.01))
                    new_accz.append(self.IMUdata.accz[i - 1]+(self.IMUdata.accz[i]-self.IMUdata.accz[i - 1])*j/int(differenz/0.01))
                    new_gyrox.append(self.IMUdata.gyrox[i - 1]+(self.IMUdata.gyrox[i]-self.IMUdata.gyrox[i - 1])*j/int(differenz/0.01))
                    new_gyroy.append(self.IMUdata.gyroy[i - 1]+(self.IMUdata.gyroy[i]-self.IMUdata.gyroy[i - 1])*j/int(differenz/0.01))
                    new_gyroz.append(self.IMUdata.gyroz[i - 1]+(self.IMUdata.gyroz[i]-self.IMUdata.gyroz[i - 1])*j/int(differenz/0.01))
            else:
                new_IMU_data_time.append(self.IMUdata.time[i - 1])
                new_accx.append(self.IMUdata.accx[i - 1])
                new_accy.append(self.IMUdata.accy[i - 1])
                new_accz.append(self.IMUdata.accz[i - 1])
                new_gyrox.append(self.IMUdata.gyrox[i - 1])
                new_gyroy.append(self.IMUdata.gyroy[i - 1])
                new_gyroz.append(self.IMUdata.gyroz[i - 1])
            differenzen.append(differenz)

        self.IMUdata.time = np.array(new_IMU_data_time)
        self.IMUdata.accx = np.array(new_accx)
        self.IMUdata.accy = np.array(new_accy)
        self.IMUdata.accz = np.array(new_accz)
        self.IMUdata.gyrox = np.array(new_gyrox)
        self.IMUdata.gyroy = np.array(new_gyroy)
        self.IMUdata.gyroz= np.array(new_gyroz)