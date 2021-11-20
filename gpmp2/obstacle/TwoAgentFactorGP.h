/**
 *  @file  TwoAgentFactorGP.h
 *  @brief Two agents avoidance cost factor, using distance between them
 *  @author Luka Petrovic
 *  @date  Jan 23, 201
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

/**
 * unary factor for obstacle avoidance, planar version
 * template robot model version
 */
template <class ROBOT, class GPINTER>
class TwoAgentFactorGP: public gtsam::NoiseModelFactor8<
                             typename ROBOT::Pose, typename ROBOT::Velocity,
                             typename ROBOT::Pose, typename ROBOT::Velocity,
                             typename ROBOT::Pose, typename ROBOT::Velocity,
                             typename ROBOT::Pose, typename ROBOT::Velocity> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;
  typedef typename Robot::Velocity Velocity;

private:
  // typedefs
  typedef TwoAgentFactorGP This;
  typedef gtsam::NoiseModelFactor8<Pose, Velocity, Pose, Velocity, Pose,
                                   Velocity, Pose, Velocity>
      Base;

  typedef GPINTER GPBase;

  // GP interpolator
  GPBase GPbase_;

  const Robot &robot_;
  double epsilon_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;
  /* Default constructor do nothing */
  TwoAgentFactorGP() : robot_(Robot()) {}

  TwoAgentFactorGP(gtsam::Key pose1Key, gtsam::Key vel1Key, gtsam::Key pose2Key,
                   gtsam::Key vel2Key, gtsam::Key pose3Key, gtsam::Key vel3Key,
                   gtsam::Key pose4Key, gtsam::Key vel4Key, const Robot &robot,
                   double cost_sigma, double epsilon,
                   const gtsam::SharedNoiseModel &Qc_model, double delta_t,
                   double tau)
      : //, double epsilon
        Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                 cost_sigma),
             pose1Key, vel1Key, pose2Key, vel2Key, pose3Key, vel3Key, pose4Key,
             vel4Key),
        GPbase_(Qc_model, delta_t, tau), robot_(robot),
        epsilon_(epsilon) { // epsilon_(epsilon),
  }
  virtual ~TwoAgentFactorGP() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(
      const typename Robot::Pose &conf1, const typename Robot::Velocity &vel1,
      const typename Robot::Pose &conf2, const typename Robot::Velocity &vel2,
      const typename Robot::Pose &conf3, const typename Robot::Velocity &vel3,
      const typename Robot::Pose &conf4, const typename Robot::Velocity &vel4,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none,
      boost::optional<gtsam::Matrix &> H7 = boost::none,
      boost::optional<gtsam::Matrix &> H8 = boost::none) const;

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "TwoAgentFactorGP :" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor8", boost::serialization::base_object<Base>(*this));
  }
};

} // namespace gpmp2

#include <gpmp2/obstacle/TwoAgentFactorGP-inl.h>
