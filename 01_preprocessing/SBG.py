import numpy as np
import sys, os
import pathlib
import json

import matplotlib
matplotlib.use('Qt5Agg')

# current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add parent folder to path
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

# Import modules
from srcpython.GTSAMdata import *
from srcpython.SBGdata import *
from srcpython.geodetictools._04Visualization import *
from srcpython.geodetictools._01base import *
from srcpython.geodetictools._01base_classes import *

# Configuration of TS measurements
interpol = True        # Interpolation of positions to same timestamps
cut_dataset_by_TS = True

# Dataset
pathdataset = sys.argv[1]

# GPS reduction rate
gps_red_rate = sys.argv[2]

# Path to trajectory output
pathtrajectory = pathdataset + "/02_trajectory"

# Create trajectory directory
pathlib.Path( pathtrajectory ).mkdir( parents=True, exist_ok=True )

print("--------------------------------------------------------------------------------")
print("Dataset Information") 
print("| path to dataset: ", pathdataset )
print("| path to trajectories: ", pathtrajectory )
print("--------------------------------------------------------------------------------\n")

# ##################################################################################
# Read artificial GNSS gaps for dataset 
# ##################################################################################

print("--------------------------------------------------------------------------------\n")
with open(pathdataset + "01_sensordata/gnss_gap_t.json", 'r') as file:
    gnss_gap_t = json.load(file)

print("--------------------------------------------------------------------------------\n")

# ##################################################################################
# Read SBG data from binary
# ##################################################################################

SBG = SBGdata()

# ---------------------------------------------------------------------------------------------------
# 1) IMU Raw data
# ---------------------------------------------------------------------------------------------------

SBG.readIMUdata( path=pathdataset + "/01_sensordata/", time_frame="UTC" ,acc_frame="NED", gyro_frame="NED" )

# Check Data for Gaps in IMU data and fill Gaps by linear interpolation (GTSAM based on complete and equidistant IMU data)
SBG.fill_IMUdata_Gaps()

# ---------------------------------------------------------------------------------------------------
# 2) GNSS Positions
# ---------------------------------------------------------------------------------------------------

SBG.GNSSdata.xyz_frame = "UTM" # aim frame
SBG.readGNSSPosdata( path=pathdataset + "/01_sensordata/", time_frame="UTC" )

# Transform to local topocentric system
[xyz_gps_loc, T] = geotransform.utm2topocentric( SBG.GNSSdata.x, SBG.GNSSdata.y, SBG.GNSSdata.z )

SBG.GNSSdata.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/GPS_utm.txt")

SBG.GNSSdata.x = xyz_gps_loc[:,0]
SBG.GNSSdata.y = xyz_gps_loc[:,1]
SBG.GNSSdata.z = xyz_gps_loc[:,2]

# Save transformation to file
np.savetxt( pathdataset + "/01_sensordata/T_UTM2topo.txt", T, fmt='%.12f', delimiter='\t')

# ---------------------------------------------------------------------------------------------------
# 3) GNSS Heading & Pitch from two antennas
# ---------------------------------------------------------------------------------------------------

SBG.readGNSS_heading_pitch( path=pathdataset + "/01_sensordata/", time_frame="UTC", hp_frame="NED")

SBG.GNSS_headingpitch.writetofile( filename = pathdataset + "/01_sensordata/GPS_head_pitch_ned.txt" ) 

# Convert from NED to ENU frame
SBG.GNSS_headingpitch.toENU() 
#SBG.GNSS_headingpitch.writetofile( filename = pathdataset + "/01_sensordata/GPS_head_pitch_enu.txt" )

# ---------------------------------------------------------------------------------------------------
# 4) Total Station Data (if recorded for dataset)
# ---------------------------------------------------------------------------------------------------

ts_data1 = TS_data()
ts_data1.read_from_ascii( path=pathdataset + "/01_sensordata/TS/01/", fileformat="correctionPlus2" )

ts_data2 = TS_data()
ts_data2.read_from_ascii( path=pathdataset + "/01_sensordata/TS/02/", fileformat="correction" )

if interpol:
    # Interpolate TS1 and TS2 to same time stamps
    if len(ts_data1.x) <len(ts_data2.x):
        print('Interpolate Data of 2nd TS to timestamps of 1st TS')
        ts_data1 = ts_data2.interpolate_to_2nd_TS(ts_data1)
    else:
        print('Interpolate Data of 1st TS to timestamps of 2nd TS')
        ts_data2 = ts_data1.interpolate_to_2nd_TS(ts_data2)

    #Reject outliers
    ts_data1 = ts_data2.outliertest_by_baselength(ts_data1)

# Intersect TS with IMU data
ts_data1.intersect_with_IMU(SBG.IMUdata)
ts_data2.intersect_with_IMU(SBG.IMUdata)

# Heading_TS_old, Pitch_TS_old, Base_TS_old = ts_data1.orientation_from_2TS( ts_data2)
# print('Heading from TS1 and TS2: ', Heading_TS_old[100])
# print('Pitch from TS1 and TS2: ', Pitch_TS_old[100])
# print('Base from TS1 and TS2: ', Base_TS_old[100,:])

# Transform in ECEF coordinates (local topocentric)
[ts1_loc, T1] = geotransform.utm2topocentric( ts_data1.x, ts_data1.y, ts_data1.z, T)
ts_data1.x = ts1_loc[:,0]
ts_data1.y = ts1_loc[:,1]
ts_data1.z = ts1_loc[:,2]
[ts2_loc, T2] = geotransform.utm2topocentric( ts_data2.x, ts_data2.y, ts_data2.z, T)
ts_data2.x = ts2_loc[:,0]
ts_data2.y = ts2_loc[:,1]
ts_data2.z = ts2_loc[:,2]

# Estimate Heading from Total Station

if interpol:
    Heading_TS, Pitch_TS, Base_TS = ts_data1.orientation_from_2TS( ts_data2)
    # print('Heading from TS1 and TS2: ', Heading_TS[100])
    # print('Pitch from TS1 and TS2: ', Pitch_TS[100])
    # print('Base from TS1 and TS2: ', Base_TS[100,:])
else:
    ts_data1_copy = ts_data1
    ts_data2_copy = ts_data2

    if len(ts_data1_copy.x) <len(ts_data2_copy.x):
        ts_data1_copy = ts_data2_copy.interpolate_to_2nd_TS(ts_data1_copy)
    else:
        ts_data2_copy = ts_data1_copy.interpolate_to_2nd_TS(ts_data2_copy)

    Heading_TS, Pitch_TS, Base_TS = ts_data1_copy.orientation_from_2TS( ts_data2_copy)

# # Just for testing:
# SBG.GNSSdata.Use_TS_instead(ts_data2,False)
# SBG.GNSS_headingpitch.Use_TS_instead(ts_data2.time,Heading_TS,Pitch_TS,False)

# ---------------------------------------------------------------------------------------------------
# 3) SBG EKF Trajectory
# ---------------------------------------------------------------------------------------------------

SBG.trajectory.xyz_frame = "UTM"
SBG.trajectory.rpy_frame = "NED"                                                                
SBG.readSBG_EKF( path=pathdataset + "/01_sensordata/",  xyz_frame="LLH", rpy_frame="NED" )    

# ---------------------------------------------------------------------------------------------------
# 3) Intersecting of SBG GNSS, IMU and TS data
# ---------------------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------------------------
# 3.0) Cut by TS data


if cut_dataset_by_TS and len(ts_data1.x) != 0 and len(ts_data2.x) != 0:
    starttime = max(ts_data2.time[0],ts_data1.time[0]) # start time 
    endtime = min(ts_data2.time[-1],ts_data1.time[-1]) # end time

    treshold_d_time = 30

    # Test for gap in data of TS01
    for i in range(1, len(ts_data1.time)):
        diff = ts_data1.time[i] - ts_data1.time[i-1]
        endtime_01 = endtime
        if diff > treshold_d_time: # Set threshold in seconds
            endtime_01 = ts_data1.time[i-1]
            print("---------------------------------------------------------------------------------------------------")
            print('Find a gap in TS01 data after '+ str(endtime-starttime)+ ' seconds and cut data')
            print("---------------------------------------------------------------------------------------------------")
            break

    # Test for gap in data of TS02
    for i in range(1, len(ts_data2.time)):
        diff = ts_data2.time[i] - ts_data2.time[i-1]
        endtime_02 = endtime
        if diff > treshold_d_time: # Set threshold in seconds
            endtime_02 = ts_data2.time[i-1]
            print("---------------------------------------------------------------------------------------------------")
            print('Find a gap in TS02 data after '+ str(endtime-starttime)+ ' seconds and cut data')
            print("---------------------------------------------------------------------------------------------------")
            break

    endtime = min(endtime_01,endtime_02) -5 # end time - 5s to avoid systematic errors at the end

    idx_hp = np.where( ((SBG.GNSS_headingpitch.time >= starttime) & (SBG.GNSS_headingpitch.time <= endtime)) )
    idx_G = np.where( ((SBG.GNSSdata.time >= starttime) & (SBG.GNSSdata.time <= endtime)) )
    idx_I = np.where( ((SBG.IMUdata.time >= starttime) & (SBG.IMUdata.time <= endtime)) )
    idx_TS1 = np.where( ((ts_data1.time >= starttime) & (ts_data1.time <= endtime-0.5)) )
    idx_TS2 = np.where( ((ts_data2.time >= starttime) & (ts_data2.time <= endtime-0.5)) )

    SBG.GNSS_headingpitch.cut_by_Index(idx_hp[0])
    SBG.GNSSdata.cut_by_Index(idx_G[0])
    SBG.IMUdata.cut_by_Index(idx_I[0])
    ts_data1.cut_by_Index(idx_TS1[0])
    ts_data2.cut_by_Index(idx_TS2[0])

    if interpol:
        Base_TS = Base_TS[idx_TS2[0]]
        Heading_TS = Heading_TS[idx_TS2[0]]
        Pitch_TS = Pitch_TS[idx_TS2[0]]
    else:
        idx_TS_H = np.where( ((ts_data2_copy.time >= starttime) & (ts_data2_copy.time <= endtime)) )
        ts_data2_copy.cut_by_Index(idx_TS_H[0])
        Base_TS = Base_TS[idx_TS_H[0]]
        Heading_TS = Heading_TS[idx_TS_H[0]]
        Pitch_TS = Pitch_TS[idx_TS_H[0]]
else:
    # ---------------------------------------------------------------------------------------------------
    # 3.1) Cut by IMU data

    xy, x_ind, y_ind = np.intersect1d( SBG.IMUdata.time, SBG.GNSS_headingpitch.time, return_indices=True )
    # Cut IMU Data
    SBG.IMUdata.time  = SBG.IMUdata.time[x_ind[0] : x_ind[-1]]
    SBG.IMUdata.accx = SBG.IMUdata.accx[x_ind[0] : x_ind[-1]]
    SBG.IMUdata.accy = SBG.IMUdata.accy[x_ind[0] : x_ind[-1]]
    SBG.IMUdata.accz = SBG.IMUdata.accz[x_ind[0] : x_ind[-1]]
    SBG.IMUdata.gyrox = SBG.IMUdata.gyrox[x_ind[0] : x_ind[-1]]
    SBG.IMUdata.gyroy = SBG.IMUdata.gyroy[x_ind[0] : x_ind[-1]]
    SBG.IMUdata.gyroz = SBG.IMUdata.gyroz[x_ind[0] : x_ind[-1]]

    # Cut GNSS heading and pitch data
    SBG.GNSS_headingpitch.time=SBG.GNSS_headingpitch.time[y_ind[0] : y_ind[-1]]
    SBG.GNSS_headingpitch.heading=SBG.GNSS_headingpitch.heading[y_ind[0] : y_ind[-1]]
    SBG.GNSS_headingpitch.pitch=SBG.GNSS_headingpitch.pitch[y_ind[0] : y_ind[-1]]
    SBG.GNSS_headingpitch.sheading=SBG.GNSS_headingpitch.sheading[y_ind[0] : y_ind[-1]]
    SBG.GNSS_headingpitch.spitch=SBG.GNSS_headingpitch.spitch[y_ind[0] : y_ind[-1]]
    SBG.GNSS_headingpitch.state=SBG.GNSS_headingpitch.state[y_ind[0] : y_ind[-1]]
    SBG.GNSS_headingpitch.baseline=SBG.GNSS_headingpitch.baseline[y_ind[0] : y_ind[-1]]

    # ---------------------------------------------------------------------------------------------------
    # 3.2 Intersecting GNSS Heading with IMU data
    #

    xy, x_ind, y_ind = np.intersect1d( SBG.IMUdata.time, SBG.GNSSdata.time, return_indices=True )

    if y_ind[0] > y_ind[-1]:
        print('ERROR: CHECK ORDER OF mergeSBG')

    # Cut GNSS data
    SBG.GNSSdata.time=SBG.GNSSdata.time[y_ind[0] : y_ind[-1]]
    SBG.GNSSdata.x=SBG.GNSSdata.x[y_ind[0] : y_ind[-1]] 
    SBG.GNSSdata.y=SBG.GNSSdata.y[y_ind[0] : y_ind[-1]]
    SBG.GNSSdata.z=SBG.GNSSdata.z[y_ind[0] : y_ind[-1]]
    SBG.GNSSdata.sx=SBG.GNSSdata.sx[y_ind[0] : y_ind[-1]]
    SBG.GNSSdata.sy=SBG.GNSSdata.sy[y_ind[0] : y_ind[-1]]
    SBG.GNSSdata.sz=SBG.GNSSdata.sz[y_ind[0] : y_ind[-1]]
    SBG.GNSSdata.State = SBG.GNSSdata.State[y_ind[0] : y_ind[-1]]

    # --------------------------------------------------------------------------------------
    # Intersecting GNSS with TS data (if recorded)
    #

    if len(ts_data1.x) != 0:
        ts_data1.cut_by_time( t_start = SBG.IMUdata.time[0], t_end = SBG.IMUdata.time[-1] )
        ts_data1.intersect( SBG.IMUdata.time )
        print("- [INFO]: Cuttet TS1 data")

    if interpol:
        idx_TS_H = np.where( ((ts_data2.time > SBG.IMUdata.time[0]) & (ts_data2.time < SBG.IMUdata.time[-1])) )
        Base_TS = Base_TS[idx_TS_H[0]]
        Heading_TS = Heading_TS[idx_TS_H[0]]
        Pitch_TS = Pitch_TS[idx_TS_H[0]]
    else:
        ts_data2_copy.cut_by_time( t_start = SBG.IMUdata.time[0], t_end = SBG.IMUdata.time[-1] )
        ts_data2_copy.intersect( SBG.IMUdata.time )

    if len(ts_data2.x) != 0:
        ts_data2.cut_by_time( t_start = SBG.IMUdata.time[0], t_end = SBG.IMUdata.time[-1] )
        ts_data2.intersect( SBG.IMUdata.time )
        print("- [INFO]: Cuttet TS2 data")
            
# ################################################################
# Reduce gps rate
redution_rate = int(gps_red_rate)

SBG.GNSSdata.State[:] = 0
SBG.GNSSdata.State[0::redution_rate] = 7

# ################################################################
# Create artificial gps gaps

time_red = SBG.GNSSdata.time - SBG.GNSSdata.time[0]

# From .json file, time = 0 is start time

for key in gnss_gap_t:
    start_value = gnss_gap_t[key]["start"]
    end_value = gnss_gap_t[key]["end"]

    idx_out_gnss = np.where( (time_red > start_value) & (time_red < end_value) )[0]

    SBG.GNSSdata.State[idx_out_gnss] = 0

# ################################################################
# Use full gps rate and no gaps in the beginning

inittime = 10 # TODO: read from config file (.json)

idx_init = np.where(SBG.GNSSdata.time < (SBG.GNSSdata.time[0] + inittime) )[0]
SBG.GNSSdata.State[idx_init] = 7
SBG.GNSSdata.State[-1] = 7

# ----------------------------------------------------------------------------------------------------------
# 3.3 Write data to files
# 

# Write to file
SBG.GNSS_headingpitch.writetofile( filename = pathdataset + "/01_sensordata/GPS_head_pitch_enu.txt" )
SBG.GNSSdata.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/GPS.txt")
SBG.IMUdata.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/IMU.txt")
ts_data1.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/TS/01/TS_data.txt")
ts_data2.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/TS/02/TS_data.txt")
if interpol:
    np.savetxt(fname= pathdataset + "/01_sensordata/BaseTS.txt", X=np.c_[ ts_data2.time, Base_TS], delimiter=" ", fmt="%10.5f")
    np.savetxt(fname= pathdataset + "/01_sensordata/TS_head_pitch.txt", X=np.c_[ ts_data2.time, Heading_TS, Pitch_TS], delimiter=" ", fmt="%10.5f") # TODO: read function of FG for TS Heading
else:
    np.savetxt(fname= pathdataset + "/01_sensordata/BaseTS.txt", X=np.c_[ ts_data2_copy.time, Base_TS], delimiter=" ", fmt="%10.5f")
    np.savetxt(fname= pathdataset + "/01_sensordata/TS_head_pitch.txt", X=np.c_[ ts_data2_copy.time, Heading_TS, Pitch_TS], delimiter=" ", fmt="%10.5f")

# Cut SBG EKF data with IMU data
SBG.trajectory.intersect(SBG.IMUdata.time)
SBG.trajectory.write_to_file( path = pathdataset + "/02_trajectory/", filename = "T_sbg_ekf" )
SBG.trajectory.rpy_toENU() 
SBG.trajectory.write_to_file( path = pathdataset + "/02_trajectory/", filename = "T_sbg_ekf_enu" )

# ----------------------------------------------------------------------------------------------------------
# 3.4 Set up Sensor Data availability vector
# 
# measurement codes:
# 0: IMU data only
# 1: GNSS position
# 2: GNSS heading / pitch
# 3: Total Station 1 position
# 4: Total Station 2 position
# 5: Total Stations baseline or heading / pitch

idx_mat = np.zeros((len(SBG.IMUdata.time),5)) 

# 3.4.1 Intersecting of all data with IMU data

# Intersecting IMU and GNSS data
xy_imu_gnss, x_ind_imu_gnss, y_ind_imu_gnss = np.intersect1d( SBG.IMUdata.time, SBG.GNSSdata.time, return_indices=True)
idx_mat[x_ind_imu_gnss,0] = 1

# Intersecting IMU and GNSS Heading and Pitch data
xy_imu_gnss_h, x_ind_imu_gnss_h, y_ind_imu_gnss_h = np.intersect1d( SBG.IMUdata.time, SBG.GNSS_headingpitch.time, return_indices=True)
idx_mat[x_ind_imu_gnss_h,1] = 1

# Intersecting IMU and TS data
if len(ts_data1.x) != 0:
    xy_imu_ts, x_ind_imu_ts, y_ind_imu_ts = np.intersect1d( SBG.IMUdata.time, ts_data1.time, return_indices=True)
    idx_mat[x_ind_imu_ts,2] = 1

if len(ts_data2.x) != 0:
    xy_imu_ts2, x_ind_imu_ts2, y_ind_imu_ts2 = np.intersect1d( SBG.IMUdata.time, ts_data2.time, return_indices=True)
    idx_mat[x_ind_imu_ts2,3] = 1

if interpol:
    if len(ts_data1.x) != 0 and len(ts_data2.x) != 0:
        xy_imu_ts, x_ind_imu_ts, y_ind_imu_ts = np.intersect1d( SBG.IMUdata.time, ts_data1.time, return_indices=True)
        idx_mat[x_ind_imu_ts,4] = 1
else:
    if len(ts_data2_copy.x) != 0:
        xy_imu_ts2, x_ind_imu_ts2, y_ind_imu_ts2 = np.intersect1d( SBG.IMUdata.time, ts_data2_copy.time, return_indices=True)
        idx_mat[x_ind_imu_ts2,4] = 1


# Write index matrix to file
np.savetxt( fname = pathdataset + "/01_sensordata/sensor_data_idx.txt", X=idx_mat, delimiter=" ", fmt="%i")

print("--------------------------------------------------------------------------------\n")

print("--------------------------------------------------------------------------------")
print("[INFO] Initial State for Gtsam")

# ----------------------------------------------------------------------------------------------------------
# 3.5 Determine Initial State for GTSAM
# 

# TODO: Option if not interpolated
if len(ts_data2.x) != 0 and len(ts_data1.x) != 0:
    # Initial Orientaion with two TS
    idx = np.abs(ts_data2.time - ts_data1.time[10]).argmin()
    basevector= np.array([ts_data1.x[10],ts_data1.y[10],ts_data1.z[10]])-np.array([ts_data2.x[idx],ts_data2.y[idx],ts_data2.z[idx]])
    i = 10
    while np.linalg.norm(basevector)>0.97 or np.linalg.norm(basevector)<0.9: 
        i=i+1
        idx = np.abs(ts_data2.time - ts_data1.time[i]).argmin()
        basevector=np.array([ts_data1.x[i],ts_data1.y[i],ts_data1.z[i]])-np.array([ts_data2.x[idx],ts_data2.y[idx],ts_data2.z[idx]])
    Base_vec_norm = basevector/np.linalg.norm(basevector)

    Yaw_TS = np.arctan2(Base_vec_norm[1], Base_vec_norm[0]) # if utm add Meridiankovergenz
    Pitch_TS = np.arctan2(basevector[2],np.linalg.norm(basevector[0:2]))
    if np.abs( ts_data1.z[i]-ts_data2.z[idx])>0.09:
        Pitch_TS= Pitch_TS-np.arctan2( 0.2006,0.9193)
  
    
    # rpy_TS = np.array((0, Pitch_TS, Yaw_TS))

    # R_NED_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
    # R_BN = R_NED_ENU @ geobase.RotmatZ(rpy_TS[2]) @ geobase.RotmatY(rpy_TS[1]) @ geobase.RotmatX(rpy_TS[0])

    # rpy_TS = Rotmat2Euler( R_BN )

    rpy_TS = np.array((geobase.deg2rad( -180 ), Pitch_TS, Yaw_TS))

    Rot = geobase.quat2Rotmat(geobase.Euler2Quat(rpy_TS[0],rpy_TS[1],rpy_TS[2])) 

    leverarm_UTM = Rot@np.array([-0.48055,0.01595,-0.31918]) 
    print(str(leverarm_UTM))
   
    print("- [INFO] Initial Orientation (TS): ", geobase.rad2deg(rpy_TS))

    x_init_TS = ts_data2.x[0]-leverarm_UTM[0]
    y_init_TS = ts_data2.y[0]-leverarm_UTM[1]
    z_init_TS = ts_data2.z[0]-leverarm_UTM[2]

    print("- [INFO] Initial Position (TS): ", ts_data1.x[i]," ",  ts_data1.y[i]," ", ts_data1.z[i])


# Get heading from the two GNSS antennas, average from 10 seconds
idx = np.where( SBG.GNSS_headingpitch.time < SBG.GNSS_headingpitch.time[0] + 10 ) # TODO read inital time from .json config file

# Initial Orientaion
roll_init = geobase.deg2rad( -180 )
pitch_init = np.median( SBG.GNSS_headingpitch.pitch[idx] )
yaw_init = np.median( SBG.GNSS_headingpitch.heading[idx] ) 

rpy = np.array((roll_init, pitch_init, yaw_init))
# rpy_NED = np.array((roll_init, pitch_init, yaw_init))

# # convert from NED tp ENU Frame
# R_NED_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
# R_BN = R_NED_ENU @ geobase.RotmatZ(rpy_NED[2]) @ geobase.RotmatY(rpy_NED[1]) @ geobase.RotmatX(rpy_NED[0])

# # convert to quaternion
# rpy = Rotmat2Euler( R_BN )
print("- [INFO] Initial Orientation (GNSS): ", geobase.rad2deg(rpy))

x_init = SBG.GNSSdata.x[0]
y_init = SBG.GNSSdata.y[0]
z_init = SBG.GNSSdata.z[0]

print("- [INFO] Initial Position (GNSS): ", x_init," ",  y_init," ", z_init)

if len(ts_data2.x) != 0 and len(ts_data1.x) != 0:
    state = np.transpose(np.array( ( ts_data1.time[0], x_init_TS, y_init_TS, z_init_TS, 0, 0, 0, rpy_TS[0], rpy_TS[1], rpy_TS[2]) ) ) # ENU Frame
else:
    state = np.transpose(np.array( ( SBG.GNSSdata.time[0], x_init, y_init, z_init, 0, 0, 0, rpy[0], rpy[1], rpy[2] ) ) ) # ENU Frame

#state = np.transpose(np.array( ( SBG.GNSSdata.time[0], x_init, y_init, z_init, 0, 0, 0, rpy_TS[0], rpy_TS[1], rpy_TS[2]) ) ) # ENU Frame

# ======= Write Data To File ====== #
print("- [INFO] Writing initial state to file ")
np.savetxt( fname = pathdataset + "/02_trajectory/Initial.txt", X=state, newline=" ", fmt="%10.10f")
print("- [INFO] ... done ")
print("--------------------------------------------------------------------------------\n")

print("--------------------------------------------------------------------------------")
print("[INFO] Sensor Data Visualization")

tabs = PlotTabs( window_title="Sensor Data Viewer" )

# GNSS Plots
fig1 = SBG.GNSSdata.plotXY( idx_init )

fig2 = SBG.GNSSdata.plotZ( ts_data1, ts_data2 )
fig3 = SBG.GNSSdata.plotSxSySz()
fig4 = SBG.GNSSdata.plotNumberOfSattelites()
fig5 = SBG.GNSSdata.plotGNSSstate()
fig6 = SBG.GNSSdata.plotUndulation()

# GNSS Heading and Pitch
fig7 = SBG.GNSS_headingpitch.plotHP()

# IMU PLots
fig8 = SBG.IMUdata.plotAcceleration()
fig9 = SBG.IMUdata.plotAngularvelocity()

# Total station data plots
if len(ts_data1.x) != 0:
    fig10= ts_data1.plot_xy( SBG.GNSSdata )

if len(ts_data2.x) != 0:
    fig11= ts_data2.plot_xy( SBG.GNSSdata )

# Add Plots to tab plots
tabs.addPlot(title= "GNSS: Positions XY", figure=fig1 )
tabs.addPlot(title= "GNSS: Heights", figure=fig2 )
tabs.addPlot(title= "GNSS: std", figure=fig3 )
tabs.addPlot(title= "GNSS: Number of Satellites", figure=fig4 )
tabs.addPlot(title= "GNSS: State", figure=fig5 )
tabs.addPlot(title= "GNSS: Geoid Undulation", figure=fig6 )
tabs.addPlot(title= "GNSS: Heading and Pitch", figure=fig7)
tabs.addPlot(title= "IMU: Acceleration", figure=fig8)
tabs.addPlot(title= "IMU: Angular Velocity", figure=fig9)

if len(ts_data1.x) != 0:
    tabs.addPlot(title= "TS1 measurements", figure=fig10)

if len(ts_data2.x) != 0:
    tabs.addPlot(title= "TS2 measurements", figure=fig11)

tabs.show()

print("- [INFO] ... done ")
print("--------------------------------------------------------------------------------\n")