/**
 *  @file  AgentAvoidanceFactor.h
 *  @brief Avoidance cost factor that avoids another agent based on position
 *  @author Mark Finean, Luka Petrovic
 *  @date  20 November, 2021
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {


template<class ROBOT>
class AgentAvoidanceFactor : public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

 public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

 private:
  // typedefs
  typedef AgentAvoidanceFactor This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  const Robot &robot_;
  double epsilon_;
  gtsam::Point3 obstacle_position_;

 public:
  typedef boost::shared_ptr<This> shared_ptr;


  AgentAvoidanceFactor() : robot_(Robot()) {}

  AgentAvoidanceFactor(gtsam::Key poseKey, const Robot &robot,
                 double cost_sigma, double epsilon, const gtsam::Point3 obstacle_position)
      : //, double epsilon
      Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma),
                                              poseKey),
      robot_(robot), epsilon_(epsilon), obstacle_position_(obstacle_position) {}

  virtual ~AgentAvoidanceFactor() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(const Pose &conf1,
                boost::optional<gtsam::Matrix &> H1 = boost::none) const;

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
             gtsam::DefaultKeyFormatter) const {
    std::cout << s << "AgentAvoidanceFactor :" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar & boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};

} // namespace gpmp2

#include <gpmp2/obstacle/AgentAvoidanceFactor-inl.h>
