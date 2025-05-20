/**
 * @brief implementation to read measurements and store data
 * @author Felix Esser
 * 
 *
 */

#ifndef MMS03_INCLUDE_DATA_HPP
#define MMS03_INCLUDE_DATA_HPP

// GTSAM LIBRARIES
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

// C++ LIBRARIES
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>

// USING NAMESPACE
using namespace gtsam;

// MY LIBS
//#include "../include/LS.hpp"

/** Namespace that provides structures to store input and output data */
namespace Data {



// =============================================================================================
/** 1) Structs
* 
* 
*/

// IMU measurement
struct InitialState {
  double time;         // Time
  Vector3 xyz;         // Translation
  Rot3 rpy;            // vector with accelerometer measurements
  Vector3 VxVyVz;      // Velocity
  Vector3 sig_xyz;     // Std position
  Vector3 sig_vxvyvz;     // Std velocity
  Vector3 sig_rpy;     // Std rpy
};

// IMU measurement
struct ImuMeasurement {
  double time;            // Time when measurement was carried out
  double dt;              // delta time wrt precious imu measurements
  Vector3 accelerometer;  // vector with accelerometer measurements
  Vector3 gyroscope;      // vector with gyroscope measurements
};

// GNSS measurements
struct GpsMeasurement {
  double time;        // GNSS measurement time
  Vector3 xyz;        // vector with xyz measured coordinates
  Vector3 sig_xyz;  // measurement sigma
  int gps_state;
};

// GNSS measurements
struct GpsHeadingPitch {
  double time;        // GNSS measurement time
  double heading;     // heading from two antennas
  double pitch;       // measurement sigma
  double sig_heading;
  double sig_pitch;
  double state; // state of the GNSS heading observation [-1: not good, 0: good]
};

// Total Station measurements
struct TSdata {
  double time;        // GNSS measurement time
  Vector3 xyz;        // vector with xyz measured coordinates
  bool use_ts_data;   // true if data is recorded, false if not
  bool state;
};

// TS Baseline measurements
struct tsBaseline {
  double time;        // TS measurement time
  double x;     // Baseline from two TS
  double y;      
  double z; 
};

// Noise Model
struct NoiseModel{
  Vector3 sig_gps;    // Std gps position
  Vector3 sig_acc_b;  // Std acceleration bias  
  Vector3 sig_gyro_b; // Std gyroscope bias
  Vector3 sig_int;    // integration covariance
  Vector2 sig_gps_yaw_pitch; // Std gps heading and pitch angle from two antennas
  Vector3 sig_prism_measurements; // standrad deviation prism measurements
};

// =============================================================================================
/** 2) Functions
* 
* 
*/

Data::InitialState loadInitial(std::string imu_data_route);

std::vector<Data::GpsHeadingPitch> load_GPS_Heading_Pitch(std::string Gps_data_route);

std::vector<ImuMeasurement> load_ImuData(std::string imu_data_route);

std::vector<GpsMeasurement> load_GpsData(std::string Gps_data_route);

std::vector<TSdata> load_TS_data(std::string filename);

std::vector<tsBaseline> load_baseline(std::string filename);

std::vector<Vector5> load_sensor_data_idx_matrix(std::string filename);

};
#endif
