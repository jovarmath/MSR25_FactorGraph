/**
 * @brief Pose Optimization of the CP1 Robot using GNSS / IMU Measurements
 * @author Felix Esser
 * 
 *
 */

// GTSAM LIB include
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// CPP HEADERS
#include <math.h>

// CPP LIBS
#include <algorithm>
#include <iomanip>
#include <iterator>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
// LIB
#include "../include/Data.hpp"
#include "../include/FactorGraph.hpp"
#include "../include/Print.hpp"
#include "../include/TrajectoryReport.hpp"
#include "../include/GNSSHeadingFactor.h"
#include "../include/GNSSFactor.h"
#include "../include/TSFactor.h"
#include "../include/TSBaselineFactor.h"
#include "../include/IMUstochastics.hpp"
#include "../include/GPSstochastics.hpp"
#include "../include/GPSPitchHeadingstochastics.hpp"
#include "../include/TSstochastics.hpp"
#include "../include/SystemConfig.hpp"
#include "../include/TSBaselinestochastics.hpp"

// USE NAMESPACE
using namespace gtsam;

// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------

// Variable Symbols
using symbol_shorthand::B;  // Bias Key: (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L;  // Landmark Key: Point3(x,y,h)
using symbol_shorthand::V;  // Velocity key: (v_x,v_y,v_z)
using symbol_shorthand::X;  // Position Key: Pose3 (x,y,z,r,p,y)

// ############################################################################################
// #
FG::FG( const char* datapath, const char* configpath, const char* outputpath ) {

  
  datapath_ = datapath;
  configpath_ = configpath;
  outputpath_ = outputpath;

  std::cout << "_____________________________________________________________________________________ " << std::endl;
  std::cout << "| -------------------------- iSAM trajectory optimization ---------------------------|" << std::endl;
  std::cout << "| - Datapath: " << datapath_ << std::endl;
  std::cout << "| - Configpath: " << configpath_ << std::endl;
  std::cout << "| - Outputpath: " << outputpath_ << std::endl;
  std::cout << "|____________________________________________________________________________________|" << std::endl;
}

// ############################################################################################
// # Read measurements from files
void FG::read_measurements(){

  // IMU and GPS data
  IMU_ = Data::load_ImuData(datapath_ + "01_sensordata/IMU.txt" );  
  GPS_ = Data::load_GpsData( datapath_ + "01_sensordata/GPS.txt" );
  GPS_HP_ = Data::load_GPS_Heading_Pitch( datapath_ + "01_sensordata/GPS_head_pitch_enu.txt" );

  // Total Station Data (if recorded)
  if (std::filesystem::exists(datapath_ + "01_sensordata/TS/01/")) {
    TS_data_ = Data::load_TS_data( datapath_ + "01_sensordata/TS/01/" );
  }
  if (std::filesystem::exists(datapath_ + "01_sensordata/TS/02/")) {
    TS2_data_ = Data::load_TS_data( datapath_ + "01_sensordata/TS/02/" );
  }
  if (std::filesystem::exists(datapath_ + "01_sensordata/BaseTS.txt" )) {
    TS_B_ = Data::load_baseline( datapath_ + "01_sensordata/BaseTS.txt" );
  }
  

  // Index matrix
  sensor_m_idx_ = Data::load_sensor_data_idx_matrix( datapath_ + "01_sensordata/sensor_data_idx.txt" );

  std::cout << "\n ____________________________________________________ " << std::endl;
  std::cout << "| -------------- IMU data information -----------------|" << std::endl;
  std::cout << "| - IMU data ("<< IMU_.size() << "):                                    " << std::endl;
  std::cout << "|     acceleration x [0]:     " << std::fixed << std::setprecision(6) << IMU_[0].accelerometer.x() << "  [m/s^2]    " << std::endl;
  std::cout << "|     acceleration y [0]:     " << std::fixed << std::setprecision(6) << IMU_[0].accelerometer.y() << "  [m/s^2]    " << std::endl;
  std::cout << "|     acceleration z [0]:     " << std::fixed << std::setprecision(6) << IMU_[0].accelerometer.z() << "  [m/s^2]    " << std::endl;
  std::cout << "|                                                       " << std::endl;
  std::cout << "|     gyroscope x [0]:        " << std::fixed << std::setprecision(6) << IMU_[0].gyroscope.x() << "  [rad/s]      " << std::endl;
  std::cout << "|     gyroscope y [0]:        " << std::fixed << std::setprecision(6) << IMU_[0].gyroscope.y() << "  [rad/s]      " << std::endl;
  std::cout << "|     gyroscope z [0]:        " << std::fixed << std::setprecision(6) << IMU_[0].gyroscope.z() << "  [rad/s]      " << std::endl;
  std::cout << "| _____________________________________________________|\n" << std::endl;
  
  std::cout << "\n ____________________________________________________ " << std::endl;
  std::cout << "| --------------- GPS data information ----------------|" << std::endl;
  std::cout << "|                                                       " << std::endl;
  std::cout << "| - GPS data ("<< GPS_.size() << "):                                    " << std::endl;
  std::cout << "|     position x [0]:         " << std::fixed << std::setprecision(6) << GPS_[0].xyz.x() << "  [m]     " << std::endl;
  std::cout << "|     position y [0]:         " << std::fixed << std::setprecision(6) << GPS_[0].xyz.y() << "  [m]    " << std::endl;
  std::cout << "|     position z [0]:         " << std::fixed << std::setprecision(6) << GPS_[0].xyz.z() << "  [m]    " << std::endl;
  std::cout << "|                                                       " << std::endl;
  std::cout << "| - GPS HP data ("<< GPS_HP_.size() << "):" << std::endl;
  std::cout << "|     heading [0]:            " << std::fixed << std::setprecision(6) << GPS_HP_[0].heading << "  [rad]     " << std::endl;
  std::cout << "|     pitch [0]:              "  << std::fixed << std::setprecision(6) << GPS_HP_[0].pitch << "  [rad]    " << std::endl;
  std::cout << "| _____________________________________________________|\n" << std::endl;



  //std::cout << "--------------------------------------------------"<< std::endl;
  //std::cout << "---> IMU MEASUREMENTS" << std::endl;
  //std::cout << std::setprecision(14) << std::endl;
  //std::cout << "--- first line: " << imu_measurements[0].time << " " << imu_measurements[0].accelerometer[0]<< " " << imu_measurements[0].accelerometer[1]<< " " << imu_measurements[0].accelerometer[2]<< " " << 
  //          imu_measurements[0].gyroscope[0]<< " " << imu_measurements[0].gyroscope[1]<< " " << imu_measurements[0].gyroscope[2]<< std::endl;
  //std::cout << "--- # Number:" << imu_measurements.size() << std::endl;
  //std::cout << "--------------------------------------------------"<< std::endl;
  //std::cout << " "<< std::endl;

}

// ############################################################################################
// # Read measurement stochastics from file
void FG::read_measurement_stochastics() {

  // IMU
  IMUstochastic_.loadFromFile( configpath_ + "IMUparams.json" );
  IMUstochastic_.print();

  // GPS
  GPSstochastic_.loadFromFile( configpath_ + "GPSparams.json" );
  GPSstochastic_.print();

  // GPS Heading and Pitch
  GPSPitchHeadingstochastic_.loadFromFile( configpath_ + "GPSPitchHeadingparams.json" );
  GPSPitchHeadingstochastic_.print();

  // Total station
  TSstochastic_.loadFromFile( configpath_ + "TSparams.json" );
  TSstochastic_.print();

  // Total station baseline
  TSbasestochastic_.loadFromFile( configpath_ + "TSparams.json" );
  TSbasestochastic_.print();
}

// ############################################################################################
// # GPS Pitch Heading stochastic settings from file
void FG::read_system_config_file() {
  SystemConfig_.loadFromFile( configpath_ );
  SystemConfig_.print();
}

// ############################################################################################
// # Read iSAM config from file
void FG::read_iSAM_config_from_file(){
  iSAMConfig_.loadFromFile( configpath_ );
  iSAMConfig_.print();  
}

// ############################################################################################
// Read initial state from file
void FG::read_init_data() {
  INIT_ = Data::loadInitial( datapath_ );
}

// ############################################################################################
// #
Vector6 FG::get_acceleration_bias_and_std(){

  size_t i = 0;

  double sum_acc_x = 0.0, sum_acc_y = 0.0, sum_acc_z = 0.0;

  // Loop in the init time
  while (IMU_[i].time < IMU_[0].time + iSAMConfig_.initialtime) {
    sum_acc_x += IMU_[i].accelerometer.x();
    sum_acc_y += IMU_[i].accelerometer.y();
    sum_acc_z += IMU_[i].accelerometer.z();
    i++;
  }

  // Mean values (Gyro bias)
  double mean_acc_x = sum_acc_x/i, mean_acc_y = sum_acc_y/i, mean_acc_z = sum_acc_z/i;

  // Squared differences
  double sum_acc_x_squared = 0.0, sum_acc_y_squared = 0.0, sum_acc_z_squared = 0.0;

  // Compute std
  i = 0;

  while (IMU_[i].time < IMU_[0].time + iSAMConfig_.initialtime) {
    sum_acc_x_squared += pow(IMU_[i].accelerometer.x() - mean_acc_x,2);
    sum_acc_y_squared += pow(IMU_[i].accelerometer.y() - mean_acc_y,2);
    sum_acc_z_squared += pow(IMU_[i].accelerometer.z() - mean_acc_z,2);
    i++;
  }

  // Standard deviations
  double std_acc_x = sqrt(sum_acc_x_squared / i);
  double std_acc_y = sqrt(sum_acc_y_squared / i);
  double std_acc_z = sqrt(sum_acc_z_squared / i);

  std::cout << "\n ____________________________________________________ " << std::endl;
  std::cout << "| ------------ Accelerometer stochastics --------------|" << std::endl;
  std::cout << "| ax (mean): " << std::fixed << std::setprecision(6) << mean_acc_x << "  [m/s^2]     " << std::endl;
  std::cout << "| ay (mean): " << std::fixed << std::setprecision(6) << mean_acc_y << "  [m/s^2]     " << std::endl;
  std::cout << "| az (mean): " << std::fixed << std::setprecision(6) << mean_acc_z << "  [m/s^2]     \n" << std::endl;
  std::cout << "| ax (std): " << std::fixed << std::setprecision(6) << std_acc_x << "  [m/s^2]     " << std::endl;
  std::cout << "| ay (std): " << std::fixed << std::setprecision(6) << std_acc_y << "  [m/s^2]     " << std::endl;
  std::cout << "| az (std): " << std::fixed << std::setprecision(6) << std_acc_z << "  [m/s^2]     \n" << std::endl;
  std::cout << "| _____________________________________________________|\n" << std::endl;

  return Vector6(mean_acc_x, mean_acc_y, mean_acc_z, std_acc_x, std_acc_y, std_acc_z);
}

// ############################################################################################
// #
Vector6 FG::get_gyroscope_bias_and_std(){

  size_t i = 0;
  double sum_gyro_x = 0.0, sum_gyro_y = 0.0, sum_gyro_z = 0.0;

  // Compute sum of all elements
  while (IMU_[i].time < IMU_[0].time + iSAMConfig_.initialtime) {
    sum_gyro_x += IMU_[i].gyroscope.x();
    sum_gyro_y += IMU_[i].gyroscope.y();
    sum_gyro_z += IMU_[i].gyroscope.z();
    i++;
  }
  
  // Mean values (Gyro bias)
  double mean_gyro_x = sum_gyro_x/i, mean_gyro_y = sum_gyro_y/i, mean_gyro_z = sum_gyro_z/i;

  // Squared differences
  double sum_gyro_x_squared = 0.0, sum_gyro_y_squared = 0.0, sum_gyro_z_squared = 0.0;

  // Compute std
  i = 0;

  while (IMU_[i].time < IMU_[0].time + iSAMConfig_.initialtime) {
    sum_gyro_x_squared += pow(IMU_[i].gyroscope.x() - mean_gyro_x,2);
    sum_gyro_y_squared += pow(IMU_[i].gyroscope.y() - mean_gyro_y,2);
    sum_gyro_z_squared += pow(IMU_[i].gyroscope.z() - mean_gyro_z,2);
    i++;
  }

  // Standard deviations
  double std_gyro_x = sqrt(sum_gyro_x_squared / i);
  double std_gyro_y = sqrt(sum_gyro_y_squared / i);
  double std_gyro_z = sqrt(sum_gyro_z_squared / i);

  std::cout << "\n ____________________________________________________ " << std::endl;
  std::cout << "| --------------- Gyroscope stochastics ---------------|" << std::endl;
  std::cout << "| wx (mean): " << std::fixed << std::setprecision(6) << mean_gyro_x << "  [rad/s]     " << std::endl;
  std::cout << "| wy (mean): " << std::fixed << std::setprecision(6) << mean_gyro_y << "  [rad/s]     " << std::endl;
  std::cout << "| wz (mean): " << std::fixed << std::setprecision(6) << mean_gyro_z << "  [rad/s]     \n" << std::endl;
  std::cout << "| wx (std): " << std::fixed << std::setprecision(6) << std_gyro_x << "  [rad/s]     " << std::endl;
  std::cout << "| wy (std): " << std::fixed << std::setprecision(6) << std_gyro_y << "  [rad/s]     " << std::endl;
  std::cout << "| wz (std): " << std::fixed << std::setprecision(6) << std_gyro_z << "  [rad/s]     \n" << std::endl;
  std::cout << "| _____________________________________________________|\n" << std::endl;

  return Vector6( mean_gyro_x, mean_gyro_y, mean_gyro_z, std_gyro_x, std_gyro_y, std_gyro_z );
}

// ############################################################################################
// #
double FG::get_mean_imu_dt(){

  size_t i = 0;
  std::vector<double> delta_t_imu;

  // Loop in the init time
  while (IMU_[i].time < IMU_[0].time + iSAMConfig_.initialtime) {
    if (i > 0)
      {
        delta_t_imu.push_back( IMU_[i+1].time - IMU_[i].time );
      }
    i++;
  }

  double sum_time_diff = std::accumulate(delta_t_imu.begin(), delta_t_imu.end(), 0.0); 
  return sum_time_diff / delta_t_imu.size();
}

// ############################################################################################
// #
void FG::initialize(){

  // ------------------------------------------------------------------
  // 1) Accelerometer

  // Get mean gravity from initialization time
  Vector6 init_acc_ = get_acceleration_bias_and_std();

  // Mean gravity
  g0_ =  sqrt( pow(init_acc_[0],2) + pow(init_acc_[1],2) + pow(init_acc_[2],2) );

  // Set accelerometer std in init time as noise
  IMUstochastic_.sigacc.x = init_acc_[3];
  IMUstochastic_.sigacc.y = init_acc_[4];
  IMUstochastic_.sigacc.z = init_acc_[5];

  // ------------------------------------------------------------------
  // 2) Gyroscope

  // Get gyro bias from initialization time
  Vector6 init_gyro_ = get_gyroscope_bias_and_std();

  // Set initial gyro bias
  init_bias_gyro_ = Vector3(init_gyro_[0], init_gyro_[1], init_gyro_[2]);

  // Set gyroscope std in init time as noise
  IMUstochastic_.sigacc.x = init_gyro_[3];
  IMUstochastic_.sigacc.y = init_gyro_[4];
  IMUstochastic_.sigacc.z = init_gyro_[5];

  // Get IMU delta time
  imu_dt_ = get_mean_imu_dt();

  // Initialize IMU parameter of isam2
  imuParams_ = PreintegratedImuMeasurements::Params::MakeSharedU( g0_ ); // before U

  // Covariance Matrix Acceleration
  Eigen::DiagonalMatrix<double, 3> cov_acc( pow(IMUstochastic_.sigacc.x,2),  pow(IMUstochastic_.sigacc.y,2),  pow(IMUstochastic_.sigacc.z,2) );

  // Covariance Matrix Gyroscope
  Eigen::DiagonalMatrix<double, 3> cov_gyro( pow(IMUstochastic_.siggyro.x,2),  pow(IMUstochastic_.siggyro.y,2),  pow(IMUstochastic_.siggyro.z,2) );

  // Covariance Matrix Gyroscope
  Eigen::DiagonalMatrix<double, 3> cov_integration( pow(iSAMConfig_.PreintegrationSig.x(),2),  pow(iSAMConfig_.PreintegrationSig.y(),2),  pow(iSAMConfig_.PreintegrationSig.z(),2) );

  // Add to imu parameter
  imuParams_->setAccelerometerCovariance(cov_acc);
  imuParams_->setGyroscopeCovariance(cov_gyro);
  imuParams_->setIntegrationCovariance(cov_integration);
  
  // Coriolis
  imuParams_->setUse2ndOrderCoriolis( false );
}



// ############################################################################################
// # 
int FG::optimize() {

  // -----------------------------------------------------------------------
  // I. ISAM INITIALIZATION
  // -----------------------------------------------------------------------

  ISAM2Params isam_params;
  isam_params.factorization = ISAM2Params::CHOLESKY;
  isam_params.relinearizeSkip = iSAMConfig_.relinearizeSkip;
  isam_params.relinearizeThreshold = iSAMConfig_.relinearizeThreshold; 
  ISAM2 isam(isam_params);
  NonlinearFactorGraph FG;
  Values newValues;

  // -----------------------------------------------------------------------
  // II. Set Prior and Initialization Variables
  // -----------------------------------------------------------------------

  // 1) Prior Pose

  //std::cout << "START Pose Attitude: " << INIT_.rpy.rpy() << std::endl;

  Pose3 P_ = Pose3( INIT_.rpy, INIT_.xyz );
  auto SigmaP = noiseModel::Diagonal::Sigmas( (Vector6() << iSAMConfig_.InitialStateSigma_.sigrpy, iSAMConfig_.InitialStateSigma_.sigxyz).finished());

  // 2) Prior Velocity
  Vector3 V_ = INIT_.VxVyVz;
  auto SigmaV = noiseModel::Diagonal::Sigmas( iSAMConfig_.InitialStateSigma_.sigvxvyvz );

  // 3) Prior IMU Bias

  // bias noise for initialization time ONLY FOR SBG !!!
  double bias_acc_sig_x = 0.0001372931;
  double bias_acc_sig_y = 0.0001372931;
  double bias_acc_sig_z = 0.0001372931;
  double bias_gyro_sig_x = 3.3937e-05;
  double bias_gyro_sig_y = 3.3937e-05;
  double bias_gyro_sig_z = 3.3937e-05;

  // TODO change this

  float init_time = 10; 

  auto B_ = imuBias::ConstantBias( Vector3::Zero(), init_bias_gyro_);
  auto SigmaB = noiseModel::Diagonal::Sigmas( (Vector6() << bias_acc_sig_x,
                                                            bias_acc_sig_y,
                                                            bias_acc_sig_z,
                                                            bias_gyro_sig_x,
                                                            bias_gyro_sig_y,
                                                            bias_gyro_sig_z ).finished());

  // 4) Initialization IMU Preintegration
  std::shared_ptr<PreintegratedImuMeasurements> PreImuMeas_new = std::make_shared<PreintegratedImuMeasurements>( imuParams_, B_ );

  // 5) Initialization Time
  double current_time = INIT_.time; // = GPS_[0].time;

  // 6) Store graph keys and time stamps
  v_key_gen_bias_.push_back( 0 );
  v_key_gen_.push_back( 0 );

  v_time_.push_back( current_time );
  v_time_bias_.push_back( current_time );

  // 7) Insert Initialization Variables and Prior Factors in Graph
  newValues.insert( X(0), P_ );
  newValues.insert( V(0), V_ );
  newValues.insert( B(0) , B_ );
  FG.emplace_shared<PriorFactor<Pose3>>( X(0), P_, SigmaP );
  FG.emplace_shared<PriorFactor<Vector3>>( V(0), V_, SigmaV );
  FG.emplace_shared<PriorFactor<imuBias::ConstantBias>>( B(0), B_, SigmaB );

  // 8) Measurements indices for GNSS and TS data
  size_t gps_idx = 0;
  size_t gps_head_idx = 0;
  size_t ts_idx = 0;
  size_t ts2_idx = 0;
  size_t tsbase_idx = 0;

  // -----------------------------------------------------------------------
  // III. Set GNSS gap vector
  // -----------------------------------------------------------------------

  std::vector<int> gnss_gap( GPS_.size(), 1 );

  // -----------------------------------------------------------------------
  // VI. Main Measurement Loop Initialization
  // -----------------------------------------------------------------------

  float delta_t_imu = 0.0;


  float time_processed = 0.0;
  

  int imu_bias_count = 0;

  // Initialization keys
  int generator_bias = 1;
  int generator = 0;

  // Initialization Progress Bar
  float prgs = 0.0;

  // Initialization IMU bias key
  auto BiasK = B(generator_bias);
  
  // Main loop
  for (size_t m = 0; m < IMU_.size(); m++) {

    current_time = IMU_[m].time;

    // ==========================================================================
    // 1) Progress output
    
    float Prz = ((float)m / (float)IMU_.size());
    if (Prz > prgs) {
      Print::printProgress( prgs, iSAMConfig_.useTS1, iSAMConfig_.useTS2, iSAMConfig_.useTSbaseline, iSAMConfig_.useGPS, iSAMConfig_.useGPSheading);
      prgs = prgs + 0.05;
    }

    // ==========================================================================

    // ==========================================================================
    // 2) IMU Preintegration
    PreImuMeas_new->integrateMeasurement( IMU_[m].accelerometer,
                                          IMU_[m].gyroscope,
                                          imu_dt_);
    delta_t_imu += imu_dt_;

    imu_bias_count++;

    // total time processed
    time_processed += imu_dt_;

    double delta_t_imu_rounded = std::round(delta_t_imu * 100.0) / 100.0;

    // Check if pose is included
    if ( delta_t_imu_rounded == std::round(iSAMConfig_.poserate * 100.0) / 100.0 ){
      pose_insert_ = 1;
    }

    // ==========================================================================
    // IMU Bias standard deviation
    if (time_processed > init_time){
      bias_acc_sig_x = IMUstochastic_.sigaccbias.x;
      bias_acc_sig_y = IMUstochastic_.sigaccbias.y;
      bias_acc_sig_z = IMUstochastic_.sigaccbias.z;
      bias_gyro_sig_x = IMUstochastic_.siggyrobias.x;
      bias_gyro_sig_y = IMUstochastic_.siggyrobias.y;
      bias_gyro_sig_z = IMUstochastic_.siggyrobias.z;
    }
    
    // ==========================================================================
    // 3) Measurement Updates
    // - GNSS Update: sensor_data_idx[0] == 1
    // - GNSS HP Update: sensor_data_idx[1] == 1
    // - TS data: sensor_data_idx[2] == 1
    // - TS2 data: sensor_data_idx[3] == 1
    // - TS Baseline data: sensor_data_idx[4] == 1

    // Read line from measurement index matrix
    Vector5 sensor_data_idx = sensor_m_idx_[m];

    // Check measurement index and if to use 
    if ( (sensor_data_idx[0] == 1 && (iSAMConfig_.useGPS == 1)) || (sensor_data_idx[1] == 1 && (iSAMConfig_.useGPSheading == 1)) || ( (sensor_data_idx[2] == 1) && (iSAMConfig_.useTS1 == 1)) || (sensor_data_idx[3] == 1 && (iSAMConfig_.useTS2 == 1)) || (sensor_data_idx[4] == 1 && (iSAMConfig_.useTSbaseline == 1)) || (pose_insert_ == 1) )  {

      // ------------------------------------------------------------------------
      // 3.1 Insert New Pose and Velocity Variable

      // Generate new variable key
      generator++;

      auto PoseKprev = X(generator - 1);
      auto PoseK = X(generator);
      auto VelKprev = V(generator - 1);
      auto VelK = V(generator);

      v_time_.push_back( current_time );
      v_key_gen_.push_back(generator);
      
      // Add new variables to graph structure
      newValues.insert( PoseK, P_ );
      newValues.insert( VelK, V_  );

      // ------------------------------------------------------------------------
      // 5.3 Insert IMU Factor
      // ------------------------------------------------------------------------

      FG.emplace_shared<ImuFactor>( PoseKprev, VelKprev, PoseK, VelK, B(generator_bias - 1), *PreImuMeas_new );

      // ------------------------------------------------------------------------

      // ------------------------------------------------------------------------
      // 5.4 Total Station Factor (1)
      // ------------------------------------------------------------------------

      if ( (sensor_data_idx[2] == 1) && (iSAMConfig_.useTS1 == 1)) {
        
        // Prism measurement
        Point3 TS_measurement = TS_data_[ts_idx].xyz;

        // TS Noise Model
        auto SigmaTS = noiseModel::Diagonal::Sigmas( (Vector3() << TSstochastic_.sigpos.x,
                                                                   TSstochastic_.sigpos.y, 
                                                                   TSstochastic_.sigpos.z).finished()
         );

        // Insert TS factor into graph structure
        if (TS_data_[ts_idx].state == 1){
          FG.emplace_shared<TSFactor>( PoseK, TS_measurement, SigmaTS, SystemConfig_.dxyz_prism1 );
        }
        if (TS_data_[ts_idx].state == 0){
          std::cout << "Skipped TS position update" << std::endl;
        }

        // Update TS measurements index
        ts_idx++;

        // disable GNSS
        //iSAMConfig_.useGPS= 0;

        // disable GNSS heading
        //iSAMConfig_.useGPSheading  = 0;

        // ============================================================================
        // IMU Bias Factor

        auto BiasKprev = B(generator_bias - 1);
        auto BiasK = B(generator_bias);
        v_key_gen_bias_.push_back(generator_bias);
        v_time_bias_.push_back( current_time );

        // Noise Model
        SigmaB = noiseModel::Diagonal::Sigmas( (Vector6() << sqrt(imu_bias_count) * bias_acc_sig_x,
                                                             sqrt(imu_bias_count) * bias_acc_sig_y,
                                                             sqrt(imu_bias_count) * bias_acc_sig_z,
                                                             sqrt(imu_bias_count) * bias_gyro_sig_x,
                                                             sqrt(imu_bias_count) * bias_gyro_sig_y,
                                                             sqrt(imu_bias_count) * bias_gyro_sig_z).finished());                                       

        // IMU Bias factor
        FG.emplace_shared<BetweenFactor<imuBias::ConstantBias>>( BiasKprev, BiasK, imuBias::ConstantBias(), SigmaB );

        // Add variable
        newValues.insert( BiasK, B_ );

        // update bias iterator
        generator_bias += 1;

        // Variable Reset
        //delta_t_imu = 0;

        // ============================================================================
      }
      // ------------------------------------------------------------------------

      // ------------------------------------------------------------------------
      // 5.5 Total Station Factor (2)
      // ------------------------------------------------------------------------

      if ( (sensor_data_idx[3] == 1) && (iSAMConfig_.useTS2 == 1)) {
        
        // Prism measurement
        Point3 TS2_measurement = TS2_data_[ts2_idx].xyz;

        // TS Noise Model
        auto SigmaTS2 = noiseModel::Diagonal::Sigmas( (Vector3() << TSstochastic_.sigpos.x,
                                                                   TSstochastic_.sigpos.y, 
                                                                   TSstochastic_.sigpos.z).finished()
         );

        // Insert TS factor into graph structure
        if (TS2_data_[ts2_idx].state == 1){
          FG.emplace_shared<TSFactor>( PoseK, TS2_measurement, SigmaTS2, SystemConfig_.dxyz_prism2 );
        }
        if (TS2_data_[ts2_idx].state == 0){
          std::cout << "Skipped TS position update" << std::endl;
        }

        // Update TS measurements index
        ts2_idx++;

        // disable GNSS
        //iSAMConfig_.useGPS= 0;

        // disable GNSS heading
        //iSAMConfig_.useGPSheading  = 0;

        // ============================================================================
        // IMU Bias Factor

        auto BiasKprev = B(generator_bias - 1);
        auto BiasK = B(generator_bias);
        v_key_gen_bias_.push_back(generator_bias);
        v_time_bias_.push_back( current_time );

        // Noise Model
        SigmaB = noiseModel::Diagonal::Sigmas( (Vector6() << sqrt(imu_bias_count) * bias_acc_sig_x,
                                                             sqrt(imu_bias_count) * bias_acc_sig_y,
                                                             sqrt(imu_bias_count) * bias_acc_sig_z,
                                                             sqrt(imu_bias_count) * bias_gyro_sig_x,
                                                             sqrt(imu_bias_count) * bias_gyro_sig_y,
                                                             sqrt(imu_bias_count) * bias_gyro_sig_z).finished());                                       

        // IMU Bias factor
        FG.emplace_shared<BetweenFactor<imuBias::ConstantBias>>( BiasKprev, BiasK, imuBias::ConstantBias(), SigmaB );

        // Add variable
        newValues.insert( BiasK, B_ );

        // update bias iterator
        generator_bias += 1;

        // Variable Reset
        //delta_t_imu = 0;

        // ============================================================================
      }
      // ------------------------------------------------------------------------

      // ------------------------------------------------------------------------
      // 5.6 TS baseline factor
      // ------------------------------------------------------------------------

      if ( (sensor_data_idx[4] == 1) && (iSAMConfig_.useTSbaseline == 1)) {

        // TSbaseline Noise Model
        auto SigmaTSbase = noiseModel::Diagonal::Sigmas( (Vector3() << TSbasestochastic_.sigbase.x,
                                                                       TSbasestochastic_.sigbase.y, 
                                                                       TSbasestochastic_.sigbase.z).finished()
         );

        FG.emplace_shared<TSBaselineFactor>( PoseK, Eigen::Vector3d( TS_B_[tsbase_idx].x, TS_B_[tsbase_idx].y, TS_B_[tsbase_idx].z), SigmaTSbase, SystemConfig_.dxyz_prism1-SystemConfig_.dxyz_prism2  );

        //-------------------------------------------------------------------------------------------------------------------------------------
        // Update TS baseline measurements index
        tsbase_idx++;

      }

      //------------------------------------------------------------------------


      // ------------------------------------------------------------------------
      // 5.7 GNSS Factor
      // ------------------------------------------------------------------------
      
      if ( (sensor_data_idx[0] == 1) && (iSAMConfig_.useGPS == 1) ) {
        
        // add only if fix or float GNSS is estimated
        if ( (GPS_[gps_idx].gps_state == 7) || (GPS_[gps_idx].gps_state == 6) ) {

          // GNSS position Measurement
          Point3 GPS_measurement = GPS_[gps_idx].xyz;

          // GNSS noise model
          auto SigmaGPS = noiseModel::Diagonal::Sigmas( GPS_[gps_idx].sig_xyz );

          FG.emplace_shared<GNSSFactor>( PoseK, GPS_measurement, SigmaGPS, SystemConfig_.dxyz_gps1 );

          // Store GNSS measurement
          Data::GpsMeasurement gnss_i;
          gnss_i.time = IMU_[m].time;
          gnss_i.xyz = GPS_[gps_idx].xyz;
          gnss_i.sig_xyz = GPS_[gps_idx].sig_xyz;
          gnss_added_.push_back(gnss_i);

          //std::cout << "GNSS position: " << SystemConfig_.dxyz_gps1 << std::endl;


          // Update gps counter
          gps_idx++;

          // ============================================================================
          // IMU Bias Factor

          auto BiasKprev = B(generator_bias - 1);
          auto BiasK = B(generator_bias);
          v_key_gen_bias_.push_back(generator_bias);
          v_time_bias_.push_back( current_time );

          // Bias noise model
          SigmaB = noiseModel::Diagonal::Sigmas( (Vector6() << sqrt(imu_bias_count) * bias_acc_sig_x,
                                                               sqrt(imu_bias_count) * bias_acc_sig_y,
                                                               sqrt(imu_bias_count) * bias_acc_sig_z,
                                                               sqrt(imu_bias_count) * bias_gyro_sig_x,
                                                               sqrt(imu_bias_count) * bias_gyro_sig_y,
                                                               sqrt(imu_bias_count) * bias_gyro_sig_z).finished());  

          // IMU bias factor
          FG.emplace_shared<BetweenFactor<imuBias::ConstantBias>>( BiasKprev, BiasK, imuBias::ConstantBias(), SigmaB );

          // Add variable
          newValues.insert( BiasK, B_ );

          // update bias iterator
          generator_bias += 1;

          // Variable Reset
          //delta_t_imu = 0;
          // ============================================================================

        // else: skip current GNSS Measurement
        } else if (GPS_[gps_idx].gps_state == 0) {
          //std::cout << "GNSS skipped ! " << std::endl;
          gnss_gap[gps_idx] = 0;
          gps_idx++;
        }
      }
      // ------------------------------------------------------------------------

      // ------------------------------------------------------------------------
      // 5.8 GNSS heading and pitch factor
      // ------------------------------------------------------------------------

      if ( (sensor_data_idx[1] == 1) && (iSAMConfig_.useGPSheading == 1)) {
        
        // Stochastic model of the GNSS heading factor
        Eigen::VectorXd sigma_gnss_head(1);
        sigma_gnss_head[0] = GPS_HP_[gps_head_idx].sig_heading;
        //sigma_gnss_head[1] = GPS_HP_[gps_head_idx].sig_heading;
        auto SigmaGPS_head = noiseModel::Diagonal::Sigmas( sigma_gnss_head );

        Vector2 PY{GPS_HP_[gps_head_idx].pitch, GPS_HP_[gps_head_idx].heading};

        FG.emplace_shared<GNSSHeadingFactor>( PoseK, PY, SigmaGPS_head, SystemConfig_.dxyz_gps1, SystemConfig_.dxyz_gps2 );
        
        // update iterator
        gps_head_idx += 1;
      }

      // ------------------------------------------------------------------------

      // ------------------------------------------------------------------------
      // 5.9 Update Factor Graph
      // ------------------------------------------------------------------------

      if ( (up_cnt_ == iSAMConfig_.updaterate ) ) {
        isam.update(FG, newValues);  

        FG.resize(0);
        newValues.clear();

        Values Result = isam.calculateEstimate();        // Estimate Solution
        P_ = Result.at<Pose3>(PoseK);                    // Pose
        V_ = Result.at<Vector3>(VelK);                   // Velocity
        //B_ = Result.at<imuBias::ConstantBias>( BiasK );  // IMU Bias
        B_ = Result.at<imuBias::ConstantBias>( B(generator_bias - 1) );

        up_cnt_ = 0;
      }
      // ------------------------------------------------------------------------

      PreImuMeas_new = std::make_shared<PreintegratedImuMeasurements>( imuParams_, B_ ); 

      delta_t_imu = 0.0;
      pose_insert_ = 0;
      imu_bias_count = 0;

      up_cnt_++;
    }
  }
  Print::printProgress(1.0, iSAMConfig_.useTS1, iSAMConfig_.useTS2, iSAMConfig_.useTSbaseline, iSAMConfig_.useGPS, iSAMConfig_.useGPSheading);

  // -----------------------------------------------------------------------
  // V. Calculate Final Results
  // -----------------------------------------------------------------------
  
  isam.update(FG, newValues);

  // -----------------------------------------------------------------------
  // Writing Results to Files
  // -----------------------------------------------------------------------

  Report::WriteToFile( isam, v_key_gen_, v_key_gen_bias_, v_time_, v_time_bias_, outputpath_, gnss_gap, gnss_added_ );

return 1;

}


