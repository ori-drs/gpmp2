/**
 *  @file  ObstacleSDFFactor-inl.h
 *  @brief Obstacle avoidance cost factor, using 3D signed distance field
 *  @author Jing Dong
 *  @date  May 11, 2016
 **/

#include <gpmp2/obstacle/ObstacleCost.h>

using namespace std;
using namespace gtsam;


namespace gpmp2 {

/* ************************************************************************** */
template <class ROBOT>
gtsam::Vector ObstacleSDFFactor<ROBOT>::evaluateError(
    const typename Robot::Pose& conf, boost::optional<gtsam::Matrix&> H1) const {

  // if Jacobians used, initialize as zeros
  // size: arm_nr_points_ * DOF
  if (H1) *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

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

    const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

    if (H1) {
      Matrix13 Jerr_point;
      err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf_, total_eps, Jerr_point);

      // chain rules
      H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

    } else {
      err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf_, total_eps);
    }
  }

  return err;
}

template <class ROBOT>
gtsam::Vector ObstacleSDFFactor<ROBOT>::spheresInCollision(
    const typename Robot::Pose& conf) const {

  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  robot_.sphereCenters(conf, sph_centers);

  // allocate cost vector
  Vector spheres_in_collision(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

    const double sphere_radius = robot_.sphere_radius(sph_idx);
    if (hingeLossObstacleCost(sph_centers[sph_idx], sdf_, sphere_radius)>0) {
      spheres_in_collision(sph_idx) = 1;
    }
    else
    {
      spheres_in_collision(sph_idx) = 0;
    }

  }

  return spheres_in_collision;
}

}
