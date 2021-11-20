/**
 *  @file  TwoAgentFactor.h
 *  @brief Two agents avoidance cost factor, using distance between them
 *  @author Luka Petrovic
 *  @date  Jan 23, 201
 **/

#pragma once
/// include neku moju funkciju za izracun udaljenosti

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
template<class ROBOT>
class TwoAgentFactor : public gtsam::NoiseModelFactor2<typename ROBOT::Pose,
                                                       typename ROBOT::Pose> {

 public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

 private:
  // typedefs
  typedef TwoAgentFactor This;
  typedef gtsam::NoiseModelFactor2<Pose, Pose> Base;

  /**
   *
   * iz sdf gluposti u distance, a treba mi posebna funkcija za racunati
   * distance izmedju dva agenta
   */
  // obstacle cost settings
  // double epsilon_;      // distance from object that start non-zero cost

  // arm: planar one, all alpha = 0
  const Robot &robot_;
  double epsilon_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;
  /// opet konstruktor srediti, nize komentare i parametre
  /* Default constructor do nothing */
  TwoAgentFactor() : robot_(Robot()) {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   */
  TwoAgentFactor(gtsam::Key pose1Key, gtsam::Key pose2Key, const Robot &robot,
                 double cost_sigma, double epsilon)
      : //, double epsilon
      Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                               cost_sigma),
           pose1Key, pose2Key),
      robot_(robot), epsilon_(epsilon) { // epsilon_(epsilon),

    // TODO: check robot is plannar
  }
  /// odavde dolje je sve rijeseno
  virtual ~TwoAgentFactor() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector
  evaluateError(const Pose &conf1, const Pose &conf2,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const;

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
             gtsam::DefaultKeyFormatter) const {
    std::cout << s << "TwoAgentFactor :" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar & boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

} // namespace gpmp2

#include <gpmp2/obstacle/TwoAgentFactor-inl.h>
