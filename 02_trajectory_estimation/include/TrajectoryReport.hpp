/**
 * @brief Function to write graph results to file
 * @author Felix Esser
 * 
 * TODOs
 */

#ifndef MMS03_EXPORT_HPP
#define MMS03_EXPORT_HPP

// GTSAM LIBRARIES
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>

// C++ LIBRARIES
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

// My libs
#include "../include/Data.hpp"
#include "../include/FactorGraph.hpp"

// USING NAMESPACE
using namespace gtsam;

namespace Report {

void WriteToFile(const ISAM2 &isam,
                 const std::vector<int> &v_key_gen,
                 const std::vector<int> &v_key_gen_bias,
                 const std::vector<double> &time,
                 const std::vector<double> &time_bias,
                 std::string output_route,
                 std::vector<int> gnss_gap,
                 std::vector<Data::GpsMeasurement> gnss_added);

 
void MarginalsReport(const ISAM2 &isam,
                     const std::vector<int> &v_key_gen,
                     const std::vector<double> &time,
                     std::string output_route);
} 

#endif