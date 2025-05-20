/**
 *  @file   TSBaselineFactor.cpp
 *  @author Manuel Mittelstedt
 *  @brief  Implementation file for Baseline between two Prisms
 *  @date   January 08, 2025
 **/


#include "../include/TSBaselineFactor.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <unistd.h>
#include <cmath>

#include <boost/optional.hpp>
#include <Eigen/Core> //Dense



using namespace std;

namespace gtsam {

//***************************************************************************
void TSBaselineFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "TSBaselineFactor on " << keyFormatter(key())
       << "\n";
  cout << "  TS Baseline measurement: " << lxyz << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
Vector TSBaselineFactor::evaluateError(const Pose3& pose, OptionalMatrixType H) const {

  const auto& rot = pose.rotation();
  double roll = rot.roll();
  double pitch = rot.pitch();
  double yaw = rot.yaw();
  
  if (H) {

    // init H
    *H = Matrix::Zero(3, 6);


    // X
    //(*H)(0, 0) = (cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*sin(yaw))*sTb_[1]+(sin(yaw)*cos(roll)-sin(pitch)*sin(roll)*cos(yaw))*sTb_[2];  // X/dRoll
    //(*H)(0, 1) = -sin(pitch)*cos(yaw)*sTb_[0]+sin(roll)*cos(pitch)*sin(yaw)*sTb_[1]+cos(roll)*cos(pitch)*sin(yaw)*sTb_[2] ;  // X/dPitch
    (*H)(0, 2) = -cos(pitch)*sin(yaw)*sTb_[0]+(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*cos(yaw))*sTb_[1]+(-cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw))*sTb_[2];  // X/dYaw

    // Y
    //(*H)(1, 0) = (cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw))*sTb_[1]+(-cos(yaw)*cos(roll)-sin(pitch)*sin(roll)*sin(yaw))*sTb_[2];  // Y/dRoll
    //(*H)(1, 1) = -sin(pitch)*sin(yaw)*sTb_[0]+sin(roll)*cos(pitch)*sin(yaw)*sTb_[1]+cos(roll)*cos(pitch)*sin(yaw)*sTb_[2] ;  // Y/dPitch
    (*H)(1, 2) = cos(pitch)*cos(yaw)*sTb_[0]+(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))*sTb_[1]+(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))*sTb_[2]; // Y/dYaw

    // Z
    //(*H)(2, 0) = -sin(pitch)*cos(roll)*sTb_[1]-sin(pitch)*sin(roll)*sTb_[2]; // Z/dRoll
    //(*H)(2, 1) = -cos(pitch)*sTb_[0]-sin(pitch)*sin(roll)*sTb_[1]-sin(pitch)*cos(roll)*sTb_[2];  // Z/dPitch
    (*H)(2, 2) = 0.0; // Z/dYaw
  }

  Vector error =  (lxyz) - (pose.rotation() * sTb_);

  return error;
}

//***************************************************************************

}/// namespace gtsam
