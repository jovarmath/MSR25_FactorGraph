/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.h
 *  @author Frank Dellaert
 *  @brief  Header file for GPS factor
 *  @date   January 22, 2014
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <vector>

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
class GTSAM_EXPORT GNSSHeadingFactor: public NoiseModelFactor1<Pose3> {

private:

  typedef NoiseModelFactor1<Pose3> Base;

  Vector2 nT_;
  Eigen::Vector3d sTb1_; ///< GPS Leverarm1
  Eigen::Vector3d sTb2_; ///< GPS Leverarm2

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
  typedef std::shared_ptr<GNSSHeadingFactor> shared_ptr;

  /// Typedef to this class
  typedef GNSSHeadingFactor This;

  /** default constructor - only use for serialization */
  //GNSSHeadingFactor(): nT_(0, 0, 0){}

  ~GNSSHeadingFactor() override {}

  /**
   * @brief Constructor from a measurement in a Cartesian frame.
   * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
   * @param key of the Pose3 variable that will be constrained
   * @param gpsIn measurement already in correct coordinates
   * @param model Gaussian noise model
   */
  GNSSHeadingFactor(Key key, const Vector2& YP, const SharedNoiseModel& model, const Eigen::Vector3d& leverarm1, const Eigen::Vector3d& leverarm2) :
      Base(model, key), nT_(YP), sTb1_(leverarm1), sTb2_(leverarm2) {
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

  inline const Vector2 & measurementIn() const {
    return nT_;
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
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
#endif

};

} /// namespace gtsam
