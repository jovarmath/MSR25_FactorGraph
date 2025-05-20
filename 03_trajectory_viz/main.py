import numpy as np
import sys
import rasterio
import matplotlib.pyplot as plt
from rasterio.plot import show
import json
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from srcpython.geodetictools._04Visualization import *
from srcpython.geodetictools._03CoordinateTransformations import *

plt.rcParams['text.usetex'] = False 
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'DejaVu Serif'



def main():

    use_img_data = False
    show_plot = True
    dataset = pathdataset = sys.argv[1]

    pathsensordata = pathdataset + "/01_sensordata/"
    pathtrajectory = pathdataset + "/02_trajectory/"

    # Read UTM to topocentrix transforamtion from file
    Transform_ = np.loadtxt( pathsensordata + "T_UTM2topo.txt" )

    # #########################################################################
    # 1) Read GPS positions
    #
    GPS = np.loadtxt( fname = pathsensordata + "GPS.txt" )
    GPS_hp_enu = np.loadtxt( fname = pathsensordata + "GPS_head_pitch_enu.txt" )
    GPS_hp_ned = np.loadtxt( fname = pathsensordata + "GPS_head_pitch_ned.txt" )

    GPS_utm = np.loadtxt( fname = pathsensordata + "GPS_utm.txt" )

    # Transform to UTM
    GPS[:,1:4] = topocentric2utm( GPS[:,1:4], Transform_ )

    # #########################################################################
    # 1) Read Trajectories
    #
    # 
    T = np.loadtxt( fname = pathtrajectory + "/T_graph.trj", comments="#", delimiter="," )
    T = T[1:,:]
    
    # Transform to UTM
    T[:,1:4] = topocentric2utm(T[:,1:4], Transform_)

    np.savetxt( pathtrajectory + "/T_graph_utm.trj", T , fmt="%10.10f")
    write_trajectory_for_trajectopy(T,pathtrajectory,'NEW')
    
    # Reference trajectory
    #Tref = np.loadtxt( fname = pathtrajectory + "/Ref/T_graph.trj", comments="#", delimiter=" " )
    #Tref[:,9] = (Tref[:,9] + 2 * np.pi) % (2 * np.pi)
    
    # SBG ekf trajectory (already in UTM! )
    Tsbg_ekf = np.loadtxt( fname = pathtrajectory + "/T_sbg_ekf.trj", comments="#", delimiter=" " )
    Tsbg_ekf[:,9] = (Tsbg_ekf[:,9] + 2 * np.pi) % (2 * np.pi)

    # ######################################################################
    # Intersect both trajectories

    print("--------------------------------------------------------------------------------\n")
    with open(pathdataset + "01_sensordata/gnss_gap_t.json", 'r') as file:
        gnss_gap_t = json.load(file)
    print("--------------------------------------------------------------------------------\n")

    # 1) Intersect reference and FG trajectory
    #xy_idx, t_idx, t_ref_idx = np.intersect1d( T[:,0], Tref[:,0], return_indices=True )

    #Tref = Tref[t_ref_idx]
    #T = T[t_idx]

    # 2) Intersect SBG ekf and FG trajectory
    xy_idx, t_idx, t_ref_idx = np.intersect1d( T[:,0], Tsbg_ekf[:,0], return_indices=True )
    Tsbg_ekf = Tsbg_ekf[t_ref_idx]

    # Convert gtsam roll pitch yaw from ENU to NED
    for i in range(len(T)):

        rpy_ENU = np.array((T[i,7], T[i,8], T[i,9]))

        # convert from NED tp ENU Frame
        R_NED_ENU = geobase.RotmatX( geobase.deg2rad( -180 ) ) @ geobase.RotmatZ( geobase.deg2rad( -90 ) )
        R_BN = R_NED_ENU @ geobase.RotmatZ(rpy_ENU[2]) @ geobase.RotmatY(rpy_ENU[1]) @ geobase.RotmatX(rpy_ENU[0])
        
        # convert to quaternion
        rpy_NED = geobase.Rotmat2Euler( R_BN )

        # Convert yaw to 0 : 360
        rpy_NED[2] = (rpy_NED[2] + 2 * np.pi) % (2 * np.pi)

        T[i,7], T[i,8], T[i,9] = rpy_NED[0], rpy_NED[1], rpy_NED[2]


    # TODO: Interpolate FG and SBG EKF and compute error!

    if show_plot:
        # Visualization
        tabs = PlotTabs( window_title="Trajectory Data Viewer" )
        
        # UTM XY plot
        fig1, ax1 = plt.subplots(figsize=(10, 10))
        ax1.plot(GPS[:,1], GPS[:,2], ".k", markersize=8, label = "GPS")
        ax1.plot(Tsbg_ekf[:,1], Tsbg_ekf[:,2], ".b", markersize=8, label = "Reference trajectory")
        ax1.plot(T[:,1], T[:,2], ".r", markersize=8, label = "GTSAM trajectory")
        ax1.plot(GPS_utm[:,1], GPS_utm[:,2], ".g", markersize=8, label = "GPS w.o. transformation")
        
        plt.legend()

        # Yaw
        fig2, ax2 = plt.subplots(figsize=(10, 10))

        ax2.plot(GPS_hp_ned[:,0], np.rad2deg(GPS_hp_ned[:,1]), "-k", markersize=8, label = "GNSS heading NED")
        ax2.plot(GPS_hp_enu[:,0], np.rad2deg(GPS_hp_enu[:,1]), "--k", markersize=8, label = "GNSS heading ENU")
        #ax2.plot(Tref[:,0], np.rad2deg(Tref[:,9]), ".b", markersize=8, label = "Reference trajectory")
        ax2.plot(Tsbg_ekf[:,0], np.rad2deg(Tsbg_ekf[:,9]), ".g", markersize=8, label = "SBG ekf trajectory")
        ax2.plot(T[:,0], np.rad2deg(T[:,9]), ".r", markersize=8, label = "GTSAM trajectory")

        plt.legend()

        # Pitch
        fig3, ax3 = plt.subplots(figsize=(10, 10))
        #ax3.plot(Tref[:,0], np.rad2deg(Tref[:,8]), ".b", markersize=8, label = "Reference trajectory")
        ax3.plot(Tsbg_ekf[:,0], np.rad2deg(Tsbg_ekf[:,8]), ".g", markersize=8, label = "SGB ekf trajectory")
        ax3.plot(T[:,0], np.rad2deg(T[:,8]), ".r", markersize=8, label = "GTSAM trajectory")

        # Roll
        fig4, ax4 = plt.subplots(figsize=(10, 10))
        #ax4.plot(Tref[:,0], np.rad2deg(Tref[:,7]), ".b", markersize=8, label = "Reference trajectory")
        ax4.plot(T[:,0], np.rad2deg(T[:,7]), ".r", markersize=8, label = "GTSAM trajectory")
        ax4.plot(Tsbg_ekf[:,0], np.rad2deg(Tsbg_ekf[:,7]), ".g", markersize=8, label = "SGB ekf trajectory")

        # Yaw differences
        fig5, ax5 = plt.subplots(figsize=(10, 10))
        #ax5.plot(Tref[:,0], np.rad2deg(Tref[:,9]) - np.rad2deg(T[:,9]), ".b", markersize=8, label = "Reference trajectory")

        tabs.addPlot(title= "Position UTM xy", figure=fig1 )
        tabs.addPlot(title= "Attitude Yaw", figure=fig2 )
        tabs.addPlot(title= "Attitude Pitch", figure=fig3 )
        tabs.addPlot(title= "Attitude Roll", figure=fig4 )

        tabs.addPlot(title= "Yaw differnces", figure=fig5 )

        tabs.show()
    
    # Reduced time vector
    #time_vec = T[:,0]
    #time_vec = time_vec - time_vec[0]

    #for key in gnss_gap_t:
        #start_value = gnss_gap_t[key]["start"]
        #end_value = gnss_gap_t[key]["end"]

        #idx_gap_idx_i = np.where( (time_vec > start_value) & (time_vec < end_value) )[0]

        # Cut trajactory parts
        #Ti_cut = T[idx_gap_idx_i,:]
        #Tired_cut = Tref[idx_gap_idx_i,:]

        # _________________________________________________________________________________________
        # 3D postion diff
        #errPos = np.linalg.norm(Tired_cut[:, 1:4] - Ti_cut[:, 1:4], axis=1)
        #squared_errors_pos = np.square(errPos)
        #mean_squared_error_pos = np.mean(squared_errors_pos)
        #rmse_pos = np.sqrt(mean_squared_error_pos)

        # _________________________________________________________________________________________
        # 3D velocity diff
        #errVel = np.linalg.norm(Tired_cut[:, 4:7] - Ti_cut[:, 4:7], axis=1)
        #squared_errors_vel = np.square(errVel)
        #mean_squared_error_vel = np.mean(squared_errors_vel)
        #rmse_vel = np.sqrt(mean_squared_error_vel)

        # _________________________________________________________________________________________
        # 3D rotation diff
        #errRos = np.linalg.norm(Tired_cut[:, 7:10] - Ti_cut[:, 7:10], axis=1)
        #squared_errors_rot = np.square(errRos)
        #mean_squared_error_rot = np.mean(squared_errors_rot)
        #rmse_rot = np.sqrt(mean_squared_error_rot)

        #print(f"RMSE of the position: {rmse_pos}")
        #print(f"RMSE of the velocity: {rmse_vel}")
        #print(f"RMSE of the rotation: {rmse_rot*(180/np.pi)}")

        #plt.figure
        #plt.plot(Tired_cut[:,0], errPos)
        #plt.show()

    

    
    if use_img_data:
        # Open the GeoTIFF file
        location = "CampusPoppelsdorf"

        if location == "CKA_south":
            dataset = rasterio.open("/mnt/d/trajectory-estimation-data/orthophotos/CampusPoppelsdorf/orthophoto_merged.tif")
        elif location == "CampusPoppelsdorf":
            dataset = rasterio.open("/mnt/d/trajectory-estimation-data/orthophotos/CampusPoppelsdorf/orthophoto_merged.tif")

    # Plot the GeoTIFF
    fig, ax = plt.subplots(figsize=(10, 10))
    #show(dataset.read([1, 2, 3]), ax=ax)

    if use_img_data:
        show((dataset,(1, 2, 3)), ax=ax, cmap='terrain')

    # Convert UTM coordinates to pixel coordinates
    # Assume T contains UTM coordinates with columns as [easting, northing]
    # dataset.transform is an Affine transform that can be used for this conversion
    utm_coords = T[:, 1:3]  # Extracting the first two columns (easting and northing)

    #utm_coords_ref = Tref[:, 1:3] 

    # Plot the trajectory on the image
    #ax.plot(utm_coords_ref[::5,0], utm_coords_ref[::5,1], ".b", markersize=8, label = "Reference trajectory")
    ax.plot(utm_coords[::10,0], utm_coords[::10,1], ".r", markersize=8, label = "INS trajectory")
    ax.plot(utm_coords[0,0], utm_coords[0,1], "*g", markersize=14, label = "Start")
    ax.plot(utm_coords[-1,0], utm_coords[-1,1], "*r", markersize=14, label = "End")

    # Show the plot with the trajectory
    plt.xlim((utm_coords[:,0].min() - 10 , utm_coords[:,0].max() + 10 ))
    plt.ylim((utm_coords[:,1].min() - 10 , utm_coords[:,1].max() + 10 ))

    #plt.title("Trajectory at campus Poppelsdorf Bonn", fontsize=24)
    plt.xlabel("UTM East [m]", fontsize=24)
    plt.ylabel("UTM North [m]", fontsize=24)
    plt.legend(fontsize=14)

    ax.tick_params(axis='x', labelsize=14)
    ax.tick_params(axis='y', labelsize=14)

    plt.axis("equal")

    #plt.show()

    # Close the dataset
    #dataset.close()








if __name__ == "__main__":
    main()