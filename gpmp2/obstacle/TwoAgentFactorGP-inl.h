/**
 *  @file  TwoAgentFactor.h
 *  @brief Two agents avoidance cost factor, using distance between them
 *  @author Luka Petrovic
 *  @date  Jan 23, 2018
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
gtsam::Vector TwoAgentFactorGP<ROBOT, GPINTER>::evaluateError(
  const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
    const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
    const typename Robot::Pose& conf3, const typename Robot::Velocity& vel3,
    const typename Robot::Pose& conf4, const typename Robot::Velocity& vel4,
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4,
    boost::optional<gtsam::Matrix&> H5, boost::optional<gtsam::Matrix&> H6,
    boost::optional<gtsam::Matrix&> H7,
    boost::optional<gtsam::Matrix&> H8) const {

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
  if (H5)
    *H5 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  if (H6)
    *H6 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  if (H7)
    *H7 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  if (H8)
    *H8 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  const bool use_H = (H1 || H2 || H3 || H4);

  // get conf by interpolation, except last pose
  typename Robot::Pose conf_int1;
  Matrix Jconf_c1, Jconf_c2, Jconf_v1, Jconf_v2;
  if (use_H)
    conf_int1 = GPbase_.interpolatePose(conf1, vel1, conf2, vel2, Jconf_c1,
                                        Jconf_v1, Jconf_c2, Jconf_v2);
  else
    conf_int1 = GPbase_.interpolatePose(conf1, vel1, conf2, vel2);

  typename Robot::Pose conf_int2;
  Matrix Jconf_c3, Jconf_c4, Jconf_v3, Jconf_v4;
  if (use_H)
    conf_int2 = GPbase_.interpolatePose(conf3, vel3, conf4, vel4, Jconf_c3,
                                        Jconf_v3, Jconf_c4, Jconf_v4);
  else
    conf_int2 = GPbase_.interpolatePose(conf3, vel3, conf4, vel4);

  Matrix Jerr_conf1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
  Matrix Jerr_conf2 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  // run forward kinematics of this configuration
  vector<Point3> sph_centers1;
  vector<Point3> sph_centers2;
  vector<Matrix> J_px_jp1;
  vector<Matrix> J_px_jp2;

  if (H1)
    robot_.sphereCenters(conf_int1, sph_centers1, J_px_jp1);
  else
    robot_.sphereCenters(conf_int1, sph_centers1);

  if (H5)
    robot_.sphereCenters(conf_int2, sph_centers2, J_px_jp2);
  else
    robot_.sphereCenters(conf_int2, sph_centers2);

  // allocate cost vector
  Vector err(robot_.nr_body_spheres());
  double dist_agents;
  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
    double minimum_dist = 10000;
    int minimum_index = 0;

    for (size_t sph_idx2 = 0; sph_idx2 < robot_.nr_body_spheres(); sph_idx2++) {
      dist_agents =
          sqrt(pow(sph_centers1[sph_idx].x() - sph_centers2[sph_idx2].x(), 2) +
               pow(sph_centers1[sph_idx].y() - sph_centers2[sph_idx2].y(), 2) +
               pow(sph_centers1[sph_idx].z() - sph_centers2[sph_idx2].z(), 2));

      if (dist_agents < minimum_dist) {
        minimum_dist = dist_agents;
        minimum_index = sph_idx2;
      }
    }

    if (H1) {
      Matrix13 Jerr_point1;
      Matrix13 Jerr_point2;
      double no_zero_div = 0.00001;

      if (minimum_dist < epsilon_) {
        err(sph_idx) = (epsilon_ - minimum_dist);

        Jerr_point1(0) =
            (sph_centers2[minimum_index].x() - sph_centers1[sph_idx].x()) /
            (minimum_dist + no_zero_div);
        Jerr_point1(1) =
            (sph_centers2[minimum_index].y() - sph_centers1[sph_idx].y()) /
            (minimum_dist + no_zero_div);
        Jerr_point1(2) =
            (sph_centers2[minimum_index].z() - sph_centers1[sph_idx].z()) /
            (minimum_dist + no_zero_div);

      } else
        err(sph_idx) = 0;
      // chain rules
      Jerr_conf1.row(sph_idx) = Jerr_point1 * J_px_jp1[sph_idx].topRows<3>();
      // H2->row(sph_idx2) = Jerr_point2 * J_px_jp2[sph_idx2].topRows<3>();

    } else {

      if (minimum_dist < epsilon_)
        err(sph_idx) = (epsilon_ - minimum_dist);
      else
        err(sph_idx) = 0;
    }
  }
  // update jacobians
  if (use_H) {
    GPBase::updatePoseJacobians(Jerr_conf1, Jconf_c1, Jconf_v1, Jconf_c2,
                                Jconf_v2, H1, H2, H3, H4);
    GPBase::updatePoseJacobians(Jerr_conf2, Jconf_c3, Jconf_v3, Jconf_c4,
                                Jconf_v4, H5, H6, H7, H8);
  }

  return err;
}

} // namespace gpmp2
