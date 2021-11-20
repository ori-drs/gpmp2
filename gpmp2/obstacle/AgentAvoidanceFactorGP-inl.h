/**
 *  @file  AgentAvoidanceFactorGP-inl.h
 *  @brief Avoidance cost factor that avoids another agent based on position
 *  @author Mark Finean, Luka Petrovic
 *  @date  20 November, 2021
 **/

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <cmath>

using namespace std;
using namespace gtsam;

namespace gpmp2 {

/* ************************************************************************** */
template <class ROBOT, class GPINTER>
gtsam::Vector AgentAvoidanceFactorGP<ROBOT, GPINTER>::evaluateError(
  const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
    const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) const {

  // if Jacobians used, initialize as zeros
  // size: arm_nr_points_ * DOF
  if (H1)
    *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  if (H2)
    *H2 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  if (H3)
    *H3 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  if (H4)
    *H4 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  const bool use_H = (H1 || H2 || H3 || H4);

  // get conf by interpolation, except last pose
  typename Robot::Pose conf_int;
  Matrix Jconf_c1, Jconf_c2, Jconf_v1, Jconf_v2;
  if (use_H)
    conf_int = GPbase_.interpolatePose(conf1, vel1, conf2, vel2, Jconf_c1,
                                        Jconf_v1, Jconf_c2, Jconf_v2);
  else
    conf_int = GPbase_.interpolatePose(conf1, vel1, conf2, vel2);

  Matrix Jerr_conf = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  vector<Matrix> J_px_jp;

  if (H1)
    robot_.sphereCenters(conf_int, sph_centers, J_px_jp);
  else
    robot_.sphereCenters(conf_int, sph_centers);

  // allocate cost vector
  Vector err(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

    double dist =sqrt(pow(sph_centers[sph_idx].x() - obstacle_position_.x(), 2) +
                      pow(sph_centers[sph_idx].y() - obstacle_position_.y(), 2) +
                      pow(sph_centers[sph_idx].z() - obstacle_position_.z(), 2));


    if (H1) {
      Matrix13 Jerr_point;
      double no_zero_div = 0.00001;

      if (dist  < epsilon_) {
        err(sph_idx) = (epsilon_ - dist);

        Jerr_point(0) = (obstacle_position_.x() - sph_centers[sph_idx].x()) / (dist + no_zero_div);
        Jerr_point(1) = (obstacle_position_.y() - sph_centers[sph_idx].y()) / (dist + no_zero_div);
        Jerr_point(2) = (obstacle_position_.z() - sph_centers[sph_idx].z()) / (dist + no_zero_div);

      } else
        err(sph_idx) = 0;

      // chain rules
      Jerr_conf.row(sph_idx) = Jerr_point * J_px_jp[sph_idx].topRows<3>();

    } else {

      if (dist < epsilon_)
        err(sph_idx) = (epsilon_ - dist);
      else
        err(sph_idx) = 0;
    }
  }
  // update jacobians
  if (use_H) {
    GPBase::updatePoseJacobians(Jerr_conf, Jconf_c1, Jconf_v1, Jconf_c2,
                                Jconf_v2, H1, H2, H3, H4);

  }

  return err;
}

} // namespace gpmp2
