/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for GPS factor
 *  @date   January 28, 2014
 **/


#include "../include/GNSSHeadingFactor.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <unistd.h>

using namespace std;

namespace gtsam {

//***************************************************************************
void GNSSHeadingFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "GPSHeadingFactor on " << keyFormatter(key())
       << "\n";
  cout << "  GPS measurement: " << nT_ << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
//bool GPSHeadingFactor::equals(const NonlinearFactor& expected, double tol) const {
//  const This* e = dynamic_cast<const This*>(&expected);
//  return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(nT_, e->nT_, tol);
//}

//***************************************************************************
/** h(x)-z */
Vector GNSSHeadingFactor::evaluateError(const Pose3& pose, OptionalMatrixType H) const {

  // Pose Variable
  const Rot3& newR = pose.rotation();
  
  if (H) {

    // Just Yaw
    *H = Matrix::Zero(1, 6);
    (*H).middleCols(2, 1).setIdentity(1, 1);

    // TODO: Add pitch angle
    
  }

  // Rotation of the current pose and measured yaw angle
  //Rot3 R_gnss_z = Rot3::RzRyRx( pose.rotation().roll(), pose.rotation().pitch(), nT_(1) );

  //Rot3 R_gnss_z = Rot3::RzRyRx( pose.rotation().roll(), pose.rotation().pitch(), nT_(1)+asin( sin(pose.rotation().roll() - 3.14159265358979323846) *  0.2006 / 0.941));
  //Rot3 R_gnss_z = Rot3::Ypr( pose.rotation().yaw(), pose.rotation().pitch(), pose.rotation().roll() );

  //Rot3 R_gnss_z = Rot3::Ypr( nT_(1)+asin( sin(pose.rotation().roll() - 3.14159265358979323846) *  0.2006 / 0.941), pose.rotation().pitch(), pose.rotation().roll() );

  Rot3 R_gnss_z = Rot3::RzRyRx( pose.rotation().roll(), pose.rotation().pitch(), nT_(1)+asin(sin(pose.rotation().roll()-3.14159265358979323846) *  (sTb1_(2)-sTb2_(2))/(sTb2_(0)-sTb1_(0)) )); //+asin(sin(pose.rotation().roll()-3.14159265358979323846) *  0.2006/ 0.941) 

  // Error in rotation
  Vector error = R_gnss_z.localCoordinates( newR ); // 3x1

  // Error just yaw
  Eigen::VectorXd ret(1);
  ret[0] = error[2];

  return ret;
}

//***************************************************************************

}/// namespace gtsam
