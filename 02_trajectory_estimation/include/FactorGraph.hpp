/**
 * @brief 
 * @author 
 *
 */

#ifndef MMS03_INCLUDE_FACTORGRAPH_HPP
#define MMS03_INCLUDE_FACTORGRAPH_HPP

// CPP LIBS
#include <string>
#include <vector>
#include <gtsam/base/Vector.h>

// INCLUDE MY LIBS

#include "../include/TrajectoryReport.hpp"
#include "../include/Data.hpp"

// GTSAM
#include <gtsam/navigation/PreintegrationParams.h>
#include "../include/IMUstochastics.hpp"
#include "../include/GPSstochastics.hpp"
#include "../include/GPSPitchHeadingstochastics.hpp"
#include "../include/TSstochastics.hpp"
#include "../include/SystemConfig.hpp"
#include "../include/iSAMConfig.hpp"
#include "../include/TSBaselinestochastics.hpp"

class FG {
 public:

  /** Class Constructor
   * Input parameters:
   * @param SystemName
   */

  FG( const char* datapath, const char* configpath, const char* outputpath );

  void read_init_data();

  void read_measurement_stochastics();

  void read_measurements();

  int optimize();

  void initialize();
  Vector6 get_acceleration_bias_and_std();
  Vector6 get_gyroscope_bias_and_std();
  double get_mean_imu_dt();

  // Functions to read system and isam config file
  void read_system_config_file();
  void read_iSAM_config_from_file();

  std::string datapath_;
  std::string configpath_;
  std::string outputpath_;


  // IMU Settings
  std::shared_ptr<gtsam::PreintegrationParams> imuParams_;
  double imu_rate_, imu_dt_;

  // Noise Model
  Data::NoiseModel NoiseModel_;

  // Initial State
  Data::InitialState INIT_; // Inititial State

  // IMU stochastics
  IMUstochastics IMUstochastic_;
  GPSstochastics GPSstochastic_;
  GPSPitchHeadingstochastics GPSPitchHeadingstochastic_;
  SystemConfig SystemConfig_;
  TSstochastics TSstochastic_;
  TSbaselinestochastics TSbasestochastic_;
  iSAMConfig iSAMConfig_;

  // Sensor Measurements
  std::vector<Data::ImuMeasurement> IMU_;  // IMU measurements
  std::vector<Data::GpsMeasurement> GPS_;  // GNSS Position
  std::vector<Data::GpsHeadingPitch> GPS_HP_; // GNSS heading and pitch data
  std::vector<Data::TSdata> TS_data_; // Total Station data (1)
  std::vector<Data::TSdata> TS2_data_; // Total Station data (2)
  std::vector<Data::tsBaseline> TS_B_; // Total Station Baseline data 

  // Sensor Measurement Matrix
  std::vector<Vector5> sensor_m_idx_;

  //int use_ts_data_;
  
  // Gyro bias computed from the initialization
  Vector3 init_bias_gyro_;

  double g0_;

  // GNSS Leverarm
  //Vector3 GPS_lv_;

  // Prism Leverarm
  //Vector3 Prism_lv_;

  // GNSS noise model
  //Vector3 gnss_noise_;

  // TS noise model
  //Vector3 ts_noise_;

  // Use GNSS heading and Pitch factor
  //int use_gnss_head_;
  //size_t gnss_skip_;
  int gnss_use_receiver_std_;
  //int ts_data_use_;
  //int gps_data_use_;

  std::vector<Data::GpsMeasurement> gnss_added_;


  // Graph
  ISAM2 isam_;                                          
  Values values_;
  int init_time_, updaterate_, relinearizeSkip_;
  double relinearizeThreshold_;
  double poserate_;

  // update counter for measurement loop
  int up_cnt_ = 0;
  int pose_insert_ = 0;

  // Graph keys
  std::vector<double> v_time_;                             
  std::vector<double> v_time_bias_;                              
  std::vector<int> v_key_gen_;
  std::vector<int> v_key_gen_bias_;

private:


};

#endif
