/**
 *  @file  AgentAvoidanceFactor.h
 *  @brief Avoidance cost factor that avoids another agent based on position
 *  @author Mark Finean, Luka Petrovic
 *  @date  20 November, 2021
 **/

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>

#include <cmath>
using namespace std;
using namespace gtsam;

namespace gpmp2 {

/* ************************************************************************** */
template <class ROBOT>
gtsam::Vector AgentAvoidanceFactor<ROBOT>::evaluateError(
    const typename Robot::Pose& conf, 
    boost::optional<gtsam::Matrix &> H1) const {


  if (H1)
    *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  vector<Matrix> J_px_jp;

  if (H1)
    robot_.sphereCenters(conf, sph_centers, J_px_jp);
  else
    robot_.sphereCenters(conf, sph_centers);

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

      if (dist < epsilon_) {
        err(sph_idx) = (epsilon_ - dist);

        Jerr_point(0) = - (sph_centers[sph_idx].x() - obstacle_position_.x()) / (dist + no_zero_div);
        Jerr_point(1) = - (sph_centers[sph_idx].y() - obstacle_position_.y()) / (dist + no_zero_div);
        Jerr_point(2) = - (sph_centers[sph_idx].z() - obstacle_position_.z()) / (dist + no_zero_div);

      } else
        err(sph_idx) = 0;
      // chain rules
      H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx].topRows<3>();

    } else {

      if (dist < epsilon_)
        err(sph_idx) = (epsilon_ - dist);
      else
        err(sph_idx) = 0;
    }
  }
  return err;
}

} // namespace gpmp2
