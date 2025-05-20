/**
 *  @file   TSFactor.h
 *  @author Felix Esser
 *  @brief  Header file for TS factor
 *  @date   April 11, 2023
 **/


#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * Prior on position in a Cartesian frame.
 * Possibilities include:
 *   ENU: East-North-Up navigation frame at some local origin
 *   NED: North-East-Down navigation frame at some local origin
 *   ECEF: Earth-centered Earth-fixed, origin at Earth's center
 * See Farrell08book or e.g. http://www.dirsig.org/docs/new/coordinates.html
 * @addtogroup Navigation
 */
class GTSAM_EXPORT TSFactor: public NoiseModelFactor1<Pose3> {

private:

  typedef NoiseModelFactor1<Pose3> Base;

  Point3 nT_; ///< Position measurement in cartesian coordinates
  Eigen::Vector3d sTb_; ///< Total Station Leverarm to Body Frame (= IMU frame)

public:

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<TSFactor> shared_ptr;

  /// Typedef to this class
  typedef TSFactor This;

  /** default constructor - only use for serialization */
  TSFactor(): nT_(0, 0, 0), sTb_(0, 0, 0){}

  ~TSFactor() override {}

  /**
   * @brief Constructor from a measurement in a Cartesian frame.
   * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
   * @param key of the Pose3 variable that will be constrained
   * @param gpsIn measurement already in correct coordinates
   * @param model Gaussian noise model
   */
  TSFactor(Key key, const Point3& gpsIn, const SharedNoiseModel& model, const Eigen::Vector3d& leverarm) :
      Base(model, key), nT_(gpsIn), sTb_(leverarm) {
  }

  /// @return a deep copy of this factor
  //gtsam::NonlinearFactor::shared_ptr clone() const override {
  //  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
  //      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  //}


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& p,
      OptionalMatrixType H ) const override;

  inline const Point3 & measurementIn() const {
    return nT_;
  }

  /**
   *  Convenience function to estimate state at time t, given two GPS
   *  readings (in local NED Cartesian frame) bracketing t
   *  Assumes roll is zero, calculates yaw and pitch from NED1->NED2 vector.
   */
  static std::pair<Pose3, Vector3> EstimateState(double t1, const Point3& NED1,
      double t2, const Point3& NED2, double timestamp);

private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
#endif
};

} /// namespace gtsam
