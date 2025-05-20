/**
 * @brief Functions to create differenet reports
 * @author JOSE ANGEL MORAGA POSSELT, MSc STUDENT UNI BONN
 * Nov 2021
 *
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

// OWN LIB
#include "../include/Data.hpp"
#include "../include/Report.hpp"

// NAMESPACES
using namespace gtsam;

// KEY TO REFER TO VARIABLES
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L;  // Point3(x,y,h)
using symbol_shorthand::V;  // Vel   (v_x,v_y,v_z)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)