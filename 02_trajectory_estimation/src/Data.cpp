/**
 * @brief implementation to read measurements and store data
 * @author JOSE ANGEL MORAGA POSSELT, MSc STUDENT UNI BONN
 * Nov 2021
 *
 */

// GTSAM LIBRARIES
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

// C++ LIBRARIES
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// Own Lib
#include "../include/Data.hpp"
#include "../include/TrajectoryReport.hpp"

// JSON Lib
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <filesystem>
namespace fs = std::filesystem;

// NAMESPACES
using namespace gtsam;

// ------------------------------------------------------------------------------------------
// INITIAL STATE
// ------------------------------------------------------------------------------------------

// Initial State
Data::InitialState Data::loadInitial(std::string init_metadata_file) {
  std::string line;

  std::string intit_metadata_ = init_metadata_file + "02_trajectory/Initial.txt";

  // Structure to store the calibration parameters
  Data::InitialState InitialState;

  // Read GPS Calibration parameters --> working
  std::ifstream gps_metadata(intit_metadata_.c_str());
  //getline(gps_metadata, line, '\n');  // ignore the first line

  double time = 0, X = 0, Y = 0, Z = 0, Vx = 0, Vy = 0, Vz = 0, roll = 0, pitch = 0, yaw = 0;
  getline(gps_metadata, line, '\n');
  sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &time, &X, &Y, &Z, &Vx, &Vy, &Vz, &roll, &pitch, &yaw);

  InitialState.time = time;
  InitialState.xyz = Vector3( X, Y, Z );
  InitialState.VxVyVz = Vector3( Vx, Vy, Vz );
  InitialState.rpy = Rot3::Ypr( yaw, pitch, roll );

  std::cout << "Roll  = " << std::setprecision(8) << roll << std::endl;
  std::cout << "Pitch = " << std::setprecision(8) << pitch << std::endl;
  std::cout << "Yaw   = " << std::setprecision(8) << yaw << std::endl;

  std::cout << "\n ______________________________________________________ " << std::endl;
  std::cout << "| ----------- Initial states information --------------|" << std::endl;
  std::cout << "| - Position xyz:    [" << std::fixed << std::setprecision(8) << InitialState.xyz.x() << ", " << InitialState.xyz.y() << ", " << InitialState.xyz.z() << "]  [m]" << std::endl;
  std::cout << "| - Velocity xyz:    [" << std::fixed << std::setprecision(8) << InitialState.VxVyVz.x() << ", " << InitialState.VxVyVz.y() << ", " << InitialState.VxVyVz.z() << "]  [m]" << std::endl;
  std::cout << "| - Orientation rpy: [" << std::fixed << std::setprecision(8) << InitialState.rpy.roll() << ", " << InitialState.rpy.pitch() << ", " << InitialState.rpy.yaw() << "]  [rad]" << std::endl;
  std::cout << "|______________________________________________________| " << std::endl;

  return InitialState;
}


// ------------------------------------------------------------------------------------------
// IMU DATA
// ------------------------------------------------------------------------------------------

std::vector<Data::ImuMeasurement> Data::load_ImuData( std::string imu_data_route) {
  std::vector<Data::ImuMeasurement> imu_measurements;


  std::string line;

  std::string imu_data_file = imu_data_route;

  {
    std::ifstream imu_data(imu_data_file.c_str());

    double time = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;

    // READ DATA
    while (!imu_data.eof()) {
      getline(imu_data, line, '\n');

      // SBG: X: front, Y:right, Z: Downwards
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &time, &acc_x, &acc_y,
             &acc_z, &gyro_x, &gyro_y, &gyro_z);

      // CREATE STRUCTURE AND FILL IT
      Data::ImuMeasurement measurement;

      measurement.time = time;
      measurement.dt = 0;
      measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
      measurement.gyroscope = Vector3(gyro_x, gyro_y, gyro_z);

      // ADD STRUCTURE TO THE VECTOR
      imu_measurements.push_back(measurement);

    }
  } 
  return imu_measurements;
}


// ------------------------------------------------------------------------------------------
// GNSS DATA
// ------------------------------------------------------------------------------------------

std::vector<Data::GpsMeasurement> Data::load_GpsData(std::string Gps_data_route) {
  // VECTOR TO STORE RESULTS
  std::vector<Data::GpsMeasurement> gps_measurements;

  // STRING USE TO READ FILES
  std::string line;

  // READ GPS DATA
  std::string gps_data_file = Gps_data_route;

  {
    std::ifstream gps_data(gps_data_file.c_str());

    double time = 0, sig_x = 0, sig_y = 0, sig_z = 0, gps_h = 0, gps_N = 0, gps_E = 0, gps_state = 0;

    while (!gps_data.eof()) {
      getline(gps_data, line, '\n');
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &gps_E, &gps_N,
             &gps_h, &sig_x, &sig_y, &sig_z, &gps_state);

      Data::GpsMeasurement measurement;
      measurement.time = time;
      measurement.xyz = Vector3(gps_E, gps_N, gps_h);
      measurement.sig_xyz = Vector3( sig_x, sig_y, sig_z );
      measurement.gps_state = (int) gps_state;
      // ADD NEW STRUCTURE TO THE VECTOR
      gps_measurements.push_back(measurement);
    }
  }
  return gps_measurements;
}

// ------------------------------------------------------------------------------------------
// SBG GNSS HEADING AND PITCH
// ------------------------------------------------------------------------------------------

std::vector<Data::GpsHeadingPitch> Data::load_GPS_Heading_Pitch(std::string filename){

  // VECTOR TO STORE RESULTS
  std::vector<Data::GpsHeadingPitch> GpsHeadingPitch_vec;

  // STRING USE TO READ FILES
  std::string line;

  // READ GPS DATA
  std::string gps_data_file = filename;

  {
    std::ifstream gps_data(gps_data_file.c_str());

    double time = 0, heading = 0, pitch = 0, sig_heading = 0, sig_pitch = 0, state = 0;

    while (!gps_data.eof()) {
      getline(gps_data, line, '\n');
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf", &time, &heading, &pitch, &sig_heading, &sig_pitch, &state);

      Data::GpsHeadingPitch GpsHeadingPitch;
      GpsHeadingPitch.time = time;
      GpsHeadingPitch.heading = heading;
      GpsHeadingPitch.pitch = pitch;
      GpsHeadingPitch.sig_heading = sig_heading;
      GpsHeadingPitch.sig_pitch = sig_pitch;
      GpsHeadingPitch.state = state;

      // ADD NEW STRUCTURE TO THE VECTOR
      GpsHeadingPitch_vec.push_back(GpsHeadingPitch);
    }
  }
  return GpsHeadingPitch_vec;
}

// ------------------------------------------------------------------------------------------
// Total Station DATA (if available)
// ------------------------------------------------------------------------------------------

std::vector<Data::TSdata> Data::load_TS_data(std::string path){

  std::vector<Data::TSdata> ts_data_vec;

  std::string filename;

  // Find all filenames in folder with .traj
  std::string ext(".traj");

  // Iterate in directory
  for (auto &p : fs::recursive_directory_iterator(path)){

    if (p.path().extension() == ext){

      // filename
      filename = path+"/TS_data.txt";

      std::string line;

      std::string ts_data_file = filename;

      std::ifstream ts_data(ts_data_file.c_str());

      double time = 0, x = 0, y = 0, z = 0;
      int state = 0;

      while (!ts_data.eof()) {
        getline(ts_data, line, '\n');
        sscanf(line.c_str(), "%lf %lf %lf %lf %d", &time, &x, &y, &z, &state);

        Data::TSdata ts_d;
        ts_d.time = time;
        ts_d.xyz = Vector3(x, y, z);
        ts_d.use_ts_data = true;
        ts_d.state = state;

        // ADD NEW STRUCTURE TO THE VECTOR
        ts_data_vec.push_back(ts_d);
      }

      std::cout << "\n ____________________________________________________ " << std::endl;
      std::cout << "| --------- Total station data information ----------|" << std::endl;
      std::cout << "| - TS data ("<< ts_data_vec.size() << "):                                    " << std::endl;
      std::cout << "|     Position x [0]:     " << std::fixed << std::setprecision(6) << ts_data_vec[0].xyz.x() << "  [m]     " << std::endl;
      std::cout << "|     Position y [0]:     " << std::fixed << std::setprecision(6) << ts_data_vec[0].xyz.y() << "  [m]    " << std::endl;
      std::cout << "|     Position z [0]:     " << std::fixed << std::setprecision(6) << ts_data_vec[0].xyz.z() << "  [m]    " << std::endl;
      std::cout << "|                                                       " << std::endl;
      std::cout << "| ___________________________________________________|\n" << std::endl;

    }else{

      Data::TSdata ts_d;
      ts_d.time = 0;
      ts_d.xyz = Vector3(0, 0, 0); 
    }
  }
  return ts_data_vec;
}

// ------------------------------------------------------------------------------------------
// TS Baseline DATA
// ------------------------------------------------------------------------------------------

std::vector<Data::tsBaseline> Data::load_baseline(std::string filename){

  // VECTOR TO STORE RESULTS
  std::vector<Data::tsBaseline> tsBaseline_vec;

  // STRING USE TO READ FILES
  std::string line;

  // READ DATA
  std::string tsbase_data_file = filename;

  {
    std::ifstream tsbase_data(tsbase_data_file.c_str());

    double time = 0, x = 0, y = 0, z = 0;

    while (!tsbase_data.eof()) {
      getline(tsbase_data, line, '\n');
      sscanf(line.c_str(), "%lf %lf %lf %lf", &time, &x, &y, &z);

      Data::tsBaseline tsbase_d;
      tsbase_d.time = time;
      tsbase_d.x = x;
      tsbase_d.y = y;
      tsbase_d.z = z;

      // ADD NEW STRUCTURE TO THE VECTOR
      tsBaseline_vec.push_back(tsbase_d);
    }
  }

  std::cout << "--------------------------------------------------"<< std::endl;
  std::cout << "---> TS Baseline MEASUREMENT" << std::endl;
  std::cout << std::setprecision(14) << std::endl;
  std::cout << "--- " << tsBaseline_vec[0].time << " " << tsBaseline_vec[0].x<< " " << tsBaseline_vec[0].y<< " " << tsBaseline_vec[0].z<<  std::endl;
  std::cout << "--- # Number: " << tsBaseline_vec.size() << std::endl;
  std::cout << "--------------------------------------------------"<< std::endl;
  std::cout << " "<< std::endl;
  return tsBaseline_vec;
}

// ------------------------------------------------------------------------------------------
// Sensor DATA Measurement Matrix
// ------------------------------------------------------------------------------------------

std::vector<Vector5> Data::load_sensor_data_idx_matrix(std::string filename){
      
      std::vector<Vector5> idx_mat;

      std::string line;

      std::string idx_mat_data_file = filename;

      std::ifstream idx_data(idx_mat_data_file.c_str());

      double imu_gnss_idx = 0, gnss_hp_idx = 0, ts_idx = 0, ts2_idx = 0, tsbase_idx = 0;

      while (!idx_data.eof()) {
        getline(idx_data, line, '\n');
        sscanf(line.c_str(), "%lf %lf %lf %lf %lf", &imu_gnss_idx, &gnss_hp_idx, &ts_idx, &ts2_idx, &tsbase_idx);

        idx_mat.push_back(Vector5(imu_gnss_idx, gnss_hp_idx, ts_idx, ts2_idx, tsbase_idx));
      }
  return idx_mat;
}