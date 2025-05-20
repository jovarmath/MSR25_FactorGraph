/**
 * @brief Function to write graph results to file
 * @author Felix Esser
 * 
 * TODOs
 */

// GTSAM LIBRARIES
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>

// C++ LIBRARIES
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept> 

// MY OWN LIBS
#include "../include/Data.hpp"
#include "../include/TrajectoryReport.hpp"

// NAMESPACES
using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L;  // Point3(x,y,h)
using symbol_shorthand::V;  // Vel   (v_x,v_y,v_z)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void Report::MarginalsReport(const ISAM2 &isam,
                             const std::vector<int> &v_key_gen,
                             const std::vector<double> &time,
                             std::string output_route) {

  // ----------------------------------------------------------------------------
  // Pose, RPY, Velocity
  // ----------------------------------------------------------------------------
  std::string pose_xyz_std_str = output_route + "pose_xyz_std.txt";
  std::string pose_rpy_std_str = output_route + "pose_rpy_std.txt";
  std::string velo_xyz_std_str = output_route + "velo_xyz_std.txt";
  std::string bias_acc_std_str = output_route + "bias_acc_std.txt";
  std::string bias_gyro_std_str = output_route + "bias_gyro_std.txt";

  std::ofstream pose_xyz_std;
  std::ofstream pose_rpy_std;
  std::ofstream velo_xyz_std;
  std::ofstream bias_acc_std;
  std::ofstream bias_gyro_std;

  // open files
  pose_xyz_std.open(pose_xyz_std_str);
  pose_rpy_std.open(pose_rpy_std_str);
  velo_xyz_std.open(velo_xyz_std_str);
  bias_acc_std.open(bias_acc_std_str);
  bias_gyro_std.open(bias_gyro_std_str);


  for (size_t j = 0; j < v_key_gen.size() - 1; j++) {

    // Get keys
    auto pose_key = X(v_key_gen[j]);
    auto bias_key = B(v_key_gen[j]);
    auto velo_key = V(v_key_gen[j]);

	  // Get Marginals from Gtsam
    auto cov = isam.marginalCovariance(pose_key);
    auto covv = isam.marginalCovariance(velo_key);
    auto covb = isam.marginalCovariance(bias_key);

    // Get Covariance Matrices
    auto covmat_pose = cov.matrix().array();
    auto covmat_velo = covv.matrix().array();
    auto covmat_bias = covb.matrix().array();

    // Save Pose Covariance Matrices

    // XYZ
    pose_xyz_std << std::fixed << std::setprecision(4) << time[j] << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(3,3) << ", ";
    pose_xyz_std << std::setprecision(10) << covmat_pose(3,4) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(3,5) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(4,3) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(4,4) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(4,5) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(5,3) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(5,4) << ", ";
    pose_xyz_std << std::setprecision(10)  << covmat_pose(5,5);
    pose_xyz_std << "\n";

    // RPY
    pose_rpy_std << std::fixed << std::setprecision(4) << time[j] << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(0,0) << ", ";
    pose_rpy_std << std::setprecision(10) << covmat_pose(0,1) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(0,2) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(1,0) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(1,1) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(1,2) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(2,0) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(2,1) << ", ";
    pose_rpy_std << std::setprecision(10)  << covmat_pose(2,2);
    pose_rpy_std << "\n";

    // Velocity
    velo_xyz_std << std::fixed << std::setprecision(4) << time[j] << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(0,0) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(0,1) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(0,2) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(1,0) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(1,1) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(1,2) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(2,0) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(2,1) << ", ";
    velo_xyz_std << std::setprecision(10)  << covmat_velo(2,2);
    velo_xyz_std << "\n";

    // bias acc 
    bias_acc_std << std::fixed << std::setprecision(4) << time[j] << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(0,0) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(0,1) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(0,2) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(1,0) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(1,1) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(1,2) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(2,0) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(2,1) << ", ";
    bias_acc_std << std::setprecision(10)  << covmat_bias(2,2);
    bias_acc_std << "\n";

    // bias gyro
    bias_gyro_std << std::fixed << std::setprecision(4) << time[j] << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(3,3) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(3,4) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(3,5) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(4,3) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(4,4) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(4,5) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(5,3) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(5,4) << ", ";
    bias_gyro_std << std::setprecision(10)  << covmat_bias(5,5);
    bias_gyro_std << "\n";
  }

  // close file
  pose_xyz_std.close();
  pose_rpy_std.close();
  velo_xyz_std.close();
  bias_acc_std.close();
  bias_gyro_std.close();
}



// -------------------------------------------------------------------------------
// Writing iSAM results to file 
// -------------------------------------------------------------------------------

void Report::WriteToFile(const ISAM2 &isam,
                         const std::vector<int> &v_key_gen,
                         const std::vector<int> &v_key_gen_bias,
                         const std::vector<double> &time,
                         const std::vector<double> &time_bias,
                         std::string output_route,
                         std::vector<int> gnss_gap,
                         std::vector<Data::GpsMeasurement>gnss_added) {

  std::cout << " __________________________________________" << std::endl;
  std::cout << "| ---- Write trajectory data to files ---- |" << std::endl;
  std::cout << "| - Optimizing final trajectory            |" << std::endl;
  Values result = isam.calculateEstimate();

  // -----------------------------------------------------------------------------------------------
  // 1) Poses and velocities 
  // -----------------------------------------------------------------------------------------------

  std::cout << "| - Writing poses to file                  |" << std::endl;
  std::cout << "| - fname = " << output_route << std::endl;

  // Open trajectory file
  std::ofstream trajectory_file;
  std::string route_tr = output_route + "T_graph.trj";
  trajectory_file.open(route_tr);

  // Open trajectory variances file
  std::ofstream pose_std;
  std::string pose_std_str = output_route + "T_pose_std.txt";
  pose_std.open(pose_std_str);

  // Write trajectory header
  trajectory_file << "#epsg 25832\n";
  trajectory_file << "#name GTSAM trajectory\n";
  trajectory_file << "#nframe enu\n";
  trajectory_file << "#fields t,px,py,pz,vx,vy,vz,ex,ey,ez\n";

  // Write trajectory variances header
  pose_std << "#fields cov00,cov01,cov02,cov03,cov04,cov05,cov10,cov11,cov12,cov13,cov14,cov15,cov20,cov21,cov22,cov23,cov24,cov25,cov30,cov31,cov32,cov33,cov34,cov35,cov40,cov41,cov42,cov43,cov44,cov45,cov50,cov51,cov52,cov53,cov54,cov55\n";

  // Loop over all keys
  for (size_t j = 0; j < v_key_gen.size() - 1; j++) {

    // Get key IDs
    Key pose_key = X( v_key_gen[j] );
    Key velocity_key = V( v_key_gen[j] );

    // Check if key exists
    if ( result.exists(pose_key) && (result.exists(velocity_key)) ) {
      try {
        // Trajectory states
        gtsam::Pose3 pose = result.at<gtsam::Pose3>(pose_key);
        gtsam::Vector3 velocity = result.at<Vector3>(velocity_key);

        // Write data to file
        trajectory_file << std::fixed << std::setprecision(8) << time[j] << ", ";
        trajectory_file << std::setprecision(4) << pose.x() << ", "; // Translation 
        trajectory_file << std::setprecision(4) << pose.y() << ", ";
        trajectory_file << std::setprecision(4) << pose.z() << ", ";
        trajectory_file << std::setprecision(10) << velocity[0] << ", "; // Velocity 
        trajectory_file << std::setprecision(10) << velocity[1] << ", ";
        trajectory_file << std::setprecision(10) << velocity[2] << ", ";
        trajectory_file << std::setprecision(10) << pose.rotation().rpy().x() << ", "; // Rotation 
        trajectory_file << std::setprecision(10) << pose.rotation().rpy().y() << ", ";
        trajectory_file << std::setprecision(10) << pose.rotation().rpy().z();
        trajectory_file << "\n";

      } catch (const std::exception& e) {}
    }
  }

  // close files
  trajectory_file.close();

  // -----------------------------------------------------------------------------------------------
  // 2) Pose variances
  // -----------------------------------------------------------------------------------------------

  std::cout << "| - Writing pose varainces to file           |" << std::endl;

  // Loop over all keys
  for (size_t j = 0; j < v_key_gen.size() - 1; j++) {

    // Get key IDs
    Key pose_key = X( v_key_gen[j] );
    Key velocity_key = V( v_key_gen[j] );

    // Check if key exists
    if ( result.exists(pose_key) && (result.exists(velocity_key)) ) {
        try {

          auto cov = isam.marginalCovariance(pose_key);

          // Covariance matrix
          auto covmat_pose = cov.matrix().array();

          // Write
          pose_std << std::fixed << std::setprecision(4) <<  time[j] << ", ";
          pose_std << std::setprecision(10)  << covmat_pose(0,0) << ", " << covmat_pose(0,1) << ", " << covmat_pose(0,2) << ", " << covmat_pose(0,3) << ", " << covmat_pose(0,4) << ", " << covmat_pose(0,5) << ", ";
          pose_std << std::setprecision(10)  << covmat_pose(1,0) << ", " << covmat_pose(1,1) << ", " << covmat_pose(1,2) << ", " << covmat_pose(1,3) << ", " << covmat_pose(1,4) << ", " << covmat_pose(1,5) << ", ";
          pose_std << std::setprecision(10)  << covmat_pose(2,0) << ", " << covmat_pose(2,1) << ", " << covmat_pose(2,2) << ", " << covmat_pose(2,3) << ", " << covmat_pose(2,4) << ", " << covmat_pose(2,5) << ", ";
          pose_std << std::setprecision(10)  << covmat_pose(3,0) << ", " << covmat_pose(3,1) << ", " << covmat_pose(3,2) << ", " << covmat_pose(3,3) << ", " << covmat_pose(3,4) << ", " << covmat_pose(3,5) << ", ";
          pose_std << std::setprecision(10)  << covmat_pose(4,0) << ", " << covmat_pose(4,1) << ", " << covmat_pose(4,2) << ", " << covmat_pose(4,3) << ", " << covmat_pose(4,4) << ", " << covmat_pose(4,5) << ", ";
          pose_std << std::setprecision(10)  << covmat_pose(5,0) << ", " << covmat_pose(5,1) << ", " << covmat_pose(5,2) << ", " << covmat_pose(5,3) << ", " << covmat_pose(5,4) << ", " << covmat_pose(5,5);
          pose_std << "\n";

        } catch (const std::exception& e) {}
    }
  }

  pose_std.close();

  // -----------------------------------------------------------------------------------------------
  // 2) IMU bias variables and covariances 
  // -----------------------------------------------------------------------------------------------

  std::cout << "| - Writing bias to file                   |" << std::endl;

  // Open bias output file
  std::ofstream bias_output;
  std::string route_bias = output_route + "IMU_bias.txt";
  bias_output.open(route_bias);

  // Loop over all keys
  for (size_t j = 0; j < v_key_gen_bias.size() - 1; j++) {

    // Access generates variable keys
    size_t k = v_key_gen_bias[j];
    auto bias_key = B(k);

    // Bias estimate
    auto bias = result.at<imuBias::ConstantBias>(bias_key);

    // Write values to file
    bias_output << std::fixed << std::setprecision(4) << time_bias[j] << ", "; 
    bias_output << std::setprecision(10) << bias.accelerometer().x() << ", "; 
    bias_output << std::setprecision(10) << bias.accelerometer().y() << ", ";
    bias_output << std::setprecision(10) << bias.accelerometer().z() << ", ";
    bias_output << std::setprecision(10) << bias.gyroscope().x() << ", ";
    bias_output << std::setprecision(10) << bias.gyroscope().y() << ", ";
    bias_output << std::setprecision(10) << bias.gyroscope().z();
    bias_output << "\n";
  }

  bias_output.close();

  std::cout << "|__________________________________________|" << std::endl;
  
  /*
  
  
  std::ofstream pose_xyz_std;
  std::ofstream pose_rpy_std;
  std::ofstream velo_xyz_std;
  
  std::ofstream bias_acc_std;
  std::ofstream bias_gyro_std;
  std::ofstream gnss_gap_;
  std::ofstream gnss_used_;

  
  std::string route_bias = output_route + "T_bias.txt";
  std::string pose_xyz_std_str = output_route + "T_xyz_std.txt";
  std::string pose_rpy_std_str = output_route + "T_rpy_std.txt";
  std::string velo_xyz_std_str = output_route + "T_vxvyvz_std.txt";
  
  std::string bias_acc_std_str = output_route + "T_bias_acc_std.txt";
  std::string bias_gyro_std_str = output_route + "T_bias_gyro_std.txt";
  std::string gnss_gap_str = output_route + "gnss_gap.txt";
  std::string gnss_used_str = output_route + "gnss_used.txt";
  
  
  bias_output.open(route_bias);
  pose_xyz_std.open(pose_xyz_std_str);
  pose_rpy_std.open(pose_rpy_std_str);
  velo_xyz_std.open(velo_xyz_std_str);
  
  bias_acc_std.open(bias_acc_std_str);
  bias_gyro_std.open(bias_gyro_std_str);
  gnss_gap_.open(gnss_gap_str);
  gnss_used_.open(gnss_used_str);


  // Write GNSS Gap Vector to File
  //for (size_t k = 0; k < gnss_gap.size(); k++) {
  //      gnss_gap_ << std::setprecision(15) << gnss_gap[k] << "\n";
  //}
  //gnss_gap_.close();
  std::cout << "--> done" << std::endl;

  // Write GNSS Used Vector to File
  for (size_t k = 0; k < gnss_added.size(); k++) {;
        gnss_used_ << std::setprecision(15) << gnss_added[k].time << ", "  << std::setprecision(15) << gnss_added[k].xyz.x() << ", " << std::setprecision(15) << gnss_added[k].xyz.y()  << ", " << std::setprecision(15) << gnss_added[k].xyz.z() << "\n";
  }
  gnss_used_.close();

  











  // =================================================================================================
  // Write IMU Bias to file
  // =================================================================================================

  std::cout << "---> Write imu bias ..." << std::endl;

  for (size_t j = 0; j < v_key_gen_bias.size(); j++) {

    // Access generates variable keys
    size_t k = v_key_gen_bias[j];

    // Bias Key
    auto bias_key = B(k);

    // =================================================================================================
    // Write IMU bias
    try {
      auto bias = result.at<imuBias::ConstantBias>(bias_key);

      // Write to file
      bias_output << std::fixed << std::setprecision(4) << time_bias[j] << ", "; // time information
      bias_output << std::setprecision(10) << bias.accelerometer().x() << ", "; // Bias Acceleration 
      bias_output << std::setprecision(10) << bias.accelerometer().y() << ", ";
      bias_output << std::setprecision(10) << bias.accelerometer().z() << ", ";
      bias_output << std::setprecision(10) << bias.gyroscope().x() << ", "; // Bias Gyroscope
      bias_output << std::setprecision(10) << bias.gyroscope().y() << ", ";
      bias_output << std::setprecision(10) << bias.gyroscope().z();
      bias_output << "\n";

    } catch (const std::exception& e) {
        std::cout << "IMU bias estimates not accessable " << e.what() << std::endl;
    }
  }

  std::cout << "Wrote IMU bias done ... " << std::endl;

  for (size_t j = 0; j < v_key_gen_bias.size(); j++) {

    // Access generates variable keys
    size_t k = v_key_gen_bias[j];

    // Bias Key
    auto bias_key = B(k);

    // =================================================================================================
    // Write IMU bias covariances
    try {

      // covariances
      auto covb = isam.marginalCovariance(bias_key);

      // to matrix
      auto covmat_bias = covb.matrix().array();

      // Variance estimates
      // bias acc 
      bias_acc_std << std::fixed << std::setprecision(4) << time_bias[j] << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(0,0) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(0,1) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(0,2) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(1,0) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(1,1) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(1,2) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(2,0) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(2,1) << ", ";
      bias_acc_std << std::setprecision(10)  << covmat_bias(2,2);
      bias_acc_std << "\n";

      // bias gyro
      bias_gyro_std << std::fixed << std::setprecision(4) << time_bias[j] << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(3,3) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(3,4) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(3,5) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(4,3) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(4,4) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(4,5) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(5,3) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(5,4) << ", ";
      bias_gyro_std << std::setprecision(10)  << covmat_bias(5,5);
      bias_gyro_std << "\n";


    } catch (const std::exception& e) {
        std::cout << "IMU bias covariances estimates not accessable " << e.what() << std::endl;
    }
  }

  // Close files
  bias_output.close();
  bias_acc_std.close();
  bias_gyro_std.close();

  std::cout << "... done" << std::endl;

  */
}