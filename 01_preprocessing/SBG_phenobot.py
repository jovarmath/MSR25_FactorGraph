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

# Convert from [-pi:pi] to [0:360]
SBG.GNSS_headingpitch.writetofile( filename = pathdataset + "/01_sensordata/GPS_head_pitch_ned.txt" )

# Convert from ENU to NED frame
SBG.GNSS_headingpitch.toENU() 

# ---------------------------------------------------------------------------------------------------
# 4) Total Station Data (if recorded for dataset)
# ---------------------------------------------------------------------------------------------------

ts_data1 = TS_data()
ts_data1.read_from_ascii( path=pathdataset + "/01_sensordata/TS/01/", fileformat="normal" )

ts_data2 = TS_data()
ts_data2.read_from_ascii( path=pathdataset + "/01_sensordata/TS/02/", fileformat="normal" )

# Transform in ECEF coordinates (local topocentric)
if len(ts_data1.x) != 0:
    [ts1_loc, T] = geotransform.utm2topocentric( ts_data1.x, ts_data1.y, ts_data1.z, T)
    ts_data1.x = ts1_loc[:,0]
    ts_data1.y = ts1_loc[:,1]
    ts_data1.z = ts1_loc[:,2]
if len(ts_data2.x) != 0:
    [ts2_loc, T] = geotransform.utm2topocentric( ts_data2.x, ts_data2.y, ts_data2.z, T)
    ts_data2.x = ts2_loc[:,0]
    ts_data2.y = ts2_loc[:,1]
    ts_data2.z = ts2_loc[:,2]

# ---------------------------------------------------------------------------------------------------
# 3) SBG EKF Trajectory
# ---------------------------------------------------------------------------------------------------

SBG.trajectory.xyz_frame = "UTM"
SBG.trajectory.rpy_frame = "END"
SBG.readSBG_EKF( path=pathdataset + "/01_sensordata/",  xyz_frame="LLH", rpy_frame="NED" )

# ---------------------------------------------------------------------------------------------------
# 3) Intersecting of SBG GNSS, IMU and TS data
# ---------------------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------------------------
# 3.0) Cut laps from dataset

cut_dataset = False

if cut_dataset:

    starttime = 0 # start time 
    endtime = 34585  # end time, individual for dataset !

    idx = np.where( ((SBG.GNSS_headingpitch.time > starttime) & (SBG.GNSS_headingpitch.time < endtime)) )
    idx = idx[0]

    SBG.GNSS_headingpitch.time=SBG.GNSS_headingpitch.time[idx]
    SBG.GNSS_headingpitch.heading=SBG.GNSS_headingpitch.heading[idx]
    SBG.GNSS_headingpitch.pitch=SBG.GNSS_headingpitch.pitch[idx]
    SBG.GNSS_headingpitch.sheading=SBG.GNSS_headingpitch.sheading[idx]
    SBG.GNSS_headingpitch.spitch=SBG.GNSS_headingpitch.spitch[idx]
    SBG.GNSS_headingpitch.state=SBG.GNSS_headingpitch.state[idx]
    SBG.GNSS_headingpitch.baseline=SBG.GNSS_headingpitch.baseline[idx]

# ---------------------------------------------------------------------------------------------------
# 3.1 Intersecting GNSS Heading with IMU data
# 
xy, x_ind, y_ind = np.intersect1d( SBG.IMUdata.time, SBG.GNSS_headingpitch.time, return_indices=True )

# Cut IMU Data
SBG.IMUdata.time  = SBG.IMUdata.time[x_ind[0] : x_ind[-1]]
SBG.IMUdata.accx = SBG.IMUdata.accx[x_ind[0] : x_ind[-1]]
SBG.IMUdata.accy = SBG.IMUdata.accy[x_ind[0] : x_ind[-1]]
SBG.IMUdata.accz = SBG.IMUdata.accz[x_ind[0] : x_ind[-1]]
SBG.IMUdata.gyrox  = SBG.IMUdata.gyrox[x_ind[0] : x_ind[-1]]
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
# 3.1 Intersecting GNSS Heading with IMU data
#

xy, x_ind, y_ind = np.intersect1d( SBG.IMUdata.time, SBG.GNSSdata.time, return_indices=True )

# Cut GNSS data
SBG.GNSSdata.time=SBG.GNSSdata.time[y_ind[0] : y_ind[-1]]
SBG.GNSSdata.x=SBG.GNSSdata.x[y_ind[0] : y_ind[-1]] 
SBG.GNSSdata.y=SBG.GNSSdata.y[y_ind[0] : y_ind[-1]]
SBG.GNSSdata.z=SBG.GNSSdata.z[y_ind[0] : y_ind[-1]]
SBG.GNSSdata.sx=SBG.GNSSdata.sx[y_ind[0] : y_ind[-1]]
SBG.GNSSdata.sy=SBG.GNSSdata.sy[y_ind[0] : y_ind[-1]]
SBG.GNSSdata.sz=SBG.GNSSdata.sz[y_ind[0] : y_ind[-1]]
SBG.GNSSdata.State = SBG.GNSSdata.State[y_ind[0] : y_ind[-1]]

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

# --------------------------------------------------------------------------------------
# 3.2 Intersecting GNSS with TS data (if recorded)
#

if len(ts_data1.x) != 0:
    ts_data1.cut_by_time( t_start = SBG.IMUdata.time[0], t_end = SBG.IMUdata.time[-1] )
    ts_data1.intersect( SBG.IMUdata.time )
    print("- [INFO]: Cuttet TS1 data")


if len(ts_data2.x) != 0:
    ts_data2.cut_by_time( t_start = SBG.IMUdata.time[0], t_end = SBG.IMUdata.time[-1] )
    ts_data2.intersect( SBG.IMUdata.time )
    print("- [INFO]: Cuttet TS2 data")

# ----------------------------------------------------------------------------------------------------------
# 3.3 Write data to files
# 

# Write to file
SBG.GNSS_headingpitch.writetofile( filename = pathdataset + "/01_sensordata/GPS_head_pitch_enu.txt" )
SBG.GNSSdata.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/GPS.txt")
SBG.IMUdata.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/IMU.txt")
ts_data1.writetofile(type="gtsam", filename = pathdataset + "/01_sensordata/TS/01/TS_data.txt")

# Cut SBG EKF data with IMU data
SBG.trajectory.intersect(SBG.IMUdata.time)
SBG.trajectory.write_to_file( path = pathdataset + "/02_trajectory/", filename = "T_sbg_ekf" )

# ----------------------------------------------------------------------------------------------------------
# 3.4 Set up Sensor Data availability vector
# 
# measurement codes:
# 0: IMU data only
# 1: GNSS position
# 2: GNSS heading / pitch
# 3: Total Station 1 position
# 4: Total Station 2 position
# 5: Total Station 12 baseline or heading / pitch

idx_mat = np.zeros((len(SBG.IMUdata.time),5)) # TODO ts2 and ts baseline / heading

# 3.4.1 Intersecting of all data with IMU data

# Intersecting IMU and GNSS data
xy_imu_gnss, x_ind_imu_gnss, y_ind_imu_gnss = np.intersect1d( SBG.IMUdata.time, SBG.GNSSdata.time, return_indices=True)
idx_mat[x_ind_imu_gnss,0] = 1

# Intersecting IMU and GNSS Heading and Pitch data
xy_imu_gnss_h, x_ind_imu_gnss_h, y_ind_imu_gnss_h = np.intersect1d( SBG.IMUdata.time, SBG.GNSS_headingpitch.time, return_indices=True)
idx_mat[x_ind_imu_gnss_h,1] = 1

# Intersecting IMU and TS1 data
if len(ts_data1.x) != 0:
    xy_imu_ts, x_ind_imu_ts, y_ind_imu_ts = np.intersect1d( SBG.IMUdata.time, ts_data1.time, return_indices=True)
    idx_mat[x_ind_imu_ts,2] = 1

# 3.4.3 Visualize measurement code vector
idx_imu = np.argwhere( idx_mat[:,0] == 0 )
idx_gnss = np.argwhere( idx_mat[:,0] == 1 )
idx_gnss_hp = np.argwhere( idx_mat[:,1] == 1 )
idx_ts = np.argwhere( idx_mat[:,2] == 1 )

time_imu = SBG.IMUdata.time[idx_imu[:,0]]
time_gnss = SBG.IMUdata.time[idx_gnss[:,0]]
time_gnss_hp = SBG.IMUdata.time[idx_gnss_hp[:,0]]
time_ts = SBG.IMUdata.time[idx_ts[:,0]]

# Write index matrix to file
np.savetxt( fname = pathdataset + "/01_sensordata/sensor_data_idx.txt", X=idx_mat, delimiter=" ", fmt="%i")

print("--------------------------------------------------------------------------------\n")

print("--------------------------------------------------------------------------------")
print("[INFO] Initial State for Gtsam")

# ----------------------------------------------------------------------------------------------------------
# 3.5 Determine Initial State for GTSAM
# 

# TODO: add option to compute initial state from ts data

# HEADING

# Get heading from the two GNSS antennas, average from 10 seconds
idx = np.where( SBG.GNSS_headingpitch.time < SBG.GNSS_headingpitch.time[0] + 10 ) # TODO read inital time from .json config file

# Initial Orientaion
roll_init = geobase.deg2rad( -180 )
pitch_init = np.median( SBG.GNSS_headingpitch.pitch[idx] )
yaw_init = np.median( SBG.GNSS_headingpitch.heading[idx] ) 

rpy = np.array((roll_init, pitch_init, yaw_init))

# # convert from NED tp ENU Frame
# R_NED_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
# R_BN = R_NED_ENU @ geobase.RotmatZ(rpy_NED[2]) @ geobase.RotmatY(rpy_NED[1]) @ geobase.RotmatX(rpy_NED[0])

# # convert to quaternion
# rpy = Rotmat2Euler( R_BN )

print("- [INFO] Initial Orientation: ", geobase.rad2deg(rpy))

x_init = SBG.GNSSdata.x[0]
y_init = SBG.GNSSdata.y[0]
z_init = SBG.GNSSdata.z[0]

print("- [INFO] Initial Position: ", x_init," ",  y_init," ", z_init)

state = np.transpose(np.array( ( SBG.GNSSdata.time[0], x_init, y_init, z_init, 0, 0, 0, rpy[0], rpy[1], rpy[2] ) ) ) # ENU Frame

# ======= Write Data To File ====== #
print("- [INFO] Writing initial state to file ")
np.savetxt( fname = pathdataset + "/02_trajectory/Initial.txt", X=state, newline=" ", fmt="%10.10f")
print("- [INFO] ... done ")
print("--------------------------------------------------------------------------------\n")

visualization = False

if visualization:
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