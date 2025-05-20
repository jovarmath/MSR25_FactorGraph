/**
 * @brief Main Pose Optimization of the CP1 Robot using GNSS / IMU Measurements
 * @author Felix Esser
 *
 *
 */

// CPP LIBRARIES
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <filesystem>

// Boost
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


// CPP HEADERS
#include <math.h>
#include <gtsam/slam/PriorFactor.h>

// LIB
#include "../include/FactorGraph.hpp"

int main( int argc, char** argv) {


  // Read input variables
  //std::string pathdataset = argv[1];
  //std::string configpath = argv[2];

  // -------------------------------------------------------------------------------
  // 1. FactorGraph Constructor
  // -------------------------------------------------------------------------------

  FG ISAM2FactorGraph( argv[1], argv[2], argv[3] );

  // argv[1]: datapath = path the data is located in
  // argv[2]: configpath = path to the configuration files
  // argv[3]: outpath = path to the folder the trajectory should be written

  std::cout << "done" << std::endl;

  // -------------------------------------------------------------------------------
  // 2. Read sensor data and stochastics
  // -------------------------------------------------------------------------------

  // 2.1 Measurements
  ISAM2FactorGraph.read_measurements();

  
  // 2.2 Stochastics
  ISAM2FactorGraph.read_measurement_stochastics();

  // -------------------------------------------------------------------------------
  // 3. Read initial state
  // -------------------------------------------------------------------------------

  ISAM2FactorGraph.read_init_data();

  // -------------------------------------------------------------------------------
  // 4. Read system config file
  // -------------------------------------------------------------------------------

  ISAM2FactorGraph.read_system_config_file();

  // -------------------------------------------------------------------------------
  // 5. Read iSAM config file
  // -------------------------------------------------------------------------------

  ISAM2FactorGraph.read_iSAM_config_from_file();

  // -------------------------------------------------------------------------------
  // 6. Initialization
  // -------------------------------------------------------------------------------

  ISAM2FactorGraph.initialize();

  // -------------------------------------------------------------------------------
  // 6. Optimization
  // -------------------------------------------------------------------------------

  int ret = ISAM2FactorGraph.optimize();
  

  return ret;
}