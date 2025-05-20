/**
 *  @file   GNSSFactor.cpp
 *  @author Felix Esser
 *  @brief  Implementation file for GNSS factor
 *  @date   April 11, 2023
 **/

#include "../include/GNSSFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void GNSSFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "GPSFactor on " << keyFormatter(key())
       << "\n";
  cout << "  GPS measurement: " << nT_ << "\n";
  noiseModel_->print("  noise model: ");
  
}

//***************************************************************************
bool GNSSFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector GNSSFactor::evaluateError(const Pose3& p, OptionalMatrixType H) const {

  //Matrix test = Matrix::Zero(3, 6);
  //test(0,0) = 1.0;
  //test(1,1) = 1.0;
  //test(2,2) = 1.0;

  //if(H) (*H) = test;

  if (H) {
    *H = Matrix::Zero(3, 6);
    //std::pair<size_t, size_t> rotInterval = pose.rotationInterval();
    //std::cout << " rotInterval " << rotInterval.first << std::endl;
    (*H).middleCols(3, 3).setIdentity(3, 3);
    //std::cout << "H row 0  = " << (*H).row(0) << std::endl;
    //std::cout << "H row 1  = " << (*H).row(1) << std::endl;
    //std::cout << "H row 2  = " << (*H).row(2) << std::endl;
  }

  //MatrixXd mat1(size, size);
  //mat1.topLeftCorner(size/2, size/2) = MatrixXd::Zero(size/2, size/2);

  Vector Error =  (p.translation(*H) + p.rotation(*H) * sTb_) - (nT_);

  
  return Error;
}

//***************************************************************************
pair<Pose3, Vector3> GNSSFactor::EstimateState(double t1, const Point3& NED1,
    double t2, const Point3& NED2, double timestamp) {
  // Estimate initial velocity as difference in NED frame
  double dt = t2 - t1;
  Point3 nV = (NED2 - NED1) / dt;

  // Estimate initial position as linear interpolation
  Point3 nT = NED1 + nV * (timestamp - t1);

  // Estimate Rotation
  double yaw = atan2(nV.y(), nV.x());
  Rot3 nRy = Rot3::Yaw(yaw); // yaw frame
  Point3 yV = nRy.inverse() * nV; // velocity in yaw frame
  double pitch = -atan2(yV.z(), yV.x()), roll = 0;
  Rot3 nRb = Rot3::Ypr(yaw, pitch, roll);

  // Construct initial pose
  Pose3 nTb(nRb, nT); // nTb

  return make_pair(nTb, nV);
}

//***************************************************************************

}/// namespace gtsam
