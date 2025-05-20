/**
 *  @file   TSBaselineFactor.h
 *  @author Manuel Mittelstedt
 *  @brief  Header file for Baseline Factor
 *  @date   January 08, 2025
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <cmath>

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
class GTSAM_EXPORT TSBaselineFactor: public NoiseModelFactor1<Pose3> {

private:

  typedef NoiseModelFactor1<Pose3> Base;

  Point3 lxyz; ///< X component of Baseline measurement of two TS
  Point3 sTb_; ///< Baseline in b-Frame
  // double ly; ///< Y component of Baseline measurement of two TS
  // double lz; ///< Z component of Baseline measurement of two TS

public:


  //typedef GNSSHeadingFactor<POSE> This;
  //typedef POSE Pose;
  //typedef typename POSE::Translation Translation;
  //typedef typename POSE::Rotation Rotation;

  //GTSAM_CONCEPT_POSE_TYPE(Pose)
  //GTSAM_CONCEPT_GROUP_TYPE(Pose)
  //GTSAM_CONCEPT_LIE_TYPE(Rotation)

  // Get dimensions of pose and rotation type at compile time
  static const int xDim = 6;
  static const int rDim = 3;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<TSBaselineFactor> shared_ptr;

  /// Typedef to this class
  typedef TSBaselineFactor This;

  /** default constructor - only use for serialization */ 
  // Baseline in b-Frame
  TSBaselineFactor(): lxyz(0, 0, 0), sTb_(0, 0, 0){}

  ~TSBaselineFactor() override {}

  /**
   * @brief Constructor from a measurement in a Cartesian frame.
   * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
   * @param key of the Pose3 variable that will be constrained
   * @param gpsIn measurement already in correct coordinates
   * @param model Gaussian noise model
   */
  TSBaselineFactor(Key key, const Point3& baseIn, const SharedNoiseModel& model, const Point3& baseinb) :
      Base(model, key), lxyz(baseIn), sTb_(baseinb) {
  }

    /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }


  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  //bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& p,
      OptionalMatrixType H) const override;

  // inline const double & measurementIn() const {
  //   return lx;
  // }
  inline Eigen::Vector3d measurementIn() const {
    return lxyz;
  }

private:



#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int ) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(lxyz);
  }
#endif

};

} /// namespace gtsam
