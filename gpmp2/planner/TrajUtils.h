/**
 *  @file  TrajUtils.h
 *  @brief utils for trajectory optimization, include initialization and interpolation
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 11, 2015
 **/

#pragma once

#include <gpmp2/config.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
// #include <stdlib.h>


namespace gpmp2 {

/**
 * @brief initial the trajectory in configuration space as a straight line
 * @param input_traj previous trajectory
 * @param start_pose  current pose
 * @param start_vel  current velocity
 * @param Qc_model  QC model
 * @param old_delta_t  delta_t in the old trajectory
 * @param new_delta_t  delta_t in the new trajectory
 * @param old_time_steps  number of time steps in old trajectory
 * @param new_time_steps  number of time steps in new trajectory
 * @param start_index The start index of the previous trajectory that you wish to keep
 * @return values
 */
GPMP2_EXPORT gtsam::Values refitPose2MobileTraj(const gtsam::Values& input_traj, const gtsam::Pose2& start_pose, const gtsam::Vector& start_vel,
        const gtsam::SharedNoiseModel Qc_model, const double old_delta_t, const double new_delta_t, const size_t old_time_steps, const size_t new_time_steps, 
        const size_t start_index);

/**
 * @brief initial the trajectory in configuration space as a straight line
 * @param init_conf trajectory start configuration
 * @param end_conf  trajectory end configuration
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
GPMP2_EXPORT gtsam::Values initArmTrajStraightLine(const gtsam::Vector& init_conf,
    const gtsam::Vector& end_conf, size_t total_step);

/**
 * @brief initial the trajectory in configuration space as random
 * @param init_conf trajectory start configuration
 * @param end_conf  trajectory end configuration
 * @param total_step is how many intervals do you want in the traj
 * @return values
 */
GPMP2_EXPORT gtsam::Values initArmTrajRandom(const gtsam::Vector& init_conf,
    const gtsam::Vector& end_conf, size_t total_step);

/**
 * @brief set the random seed used to generate trajectories
 * @param seed is how many intervals do you want in the traj
 * @return void
 */
GPMP2_EXPORT void setArmSeed(size_t seed);

/**
 * @brief reinitialise the remainder of the trajectory in configuration space as a straight line
 * @param traj trajectory start configuration
 * @param end_conf  trajectory end configuration
 * @param end_vel  trajectory end velocity
 * @param current_step is how many intervals do you want in the traj
 * @return values, straight line from current time to end
 */
GPMP2_EXPORT gtsam::Values reinitRemainderArmTrajStraightLine(gtsam::Values& traj,
    const gtsam::Vector& end_conf, const gtsam::Vector& end_vel, size_t current_step);

/**
 * @brief initialize the trajectory in configuration space as a straight line
 * @param init_pose trajectory start pose
 * @param init_conf trajectory start configuration
 * @param end_pose  trajectory end pose
 * @param end_conf  trajectory end configuration
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
GPMP2_EXPORT gtsam::Values initPose2VectorTrajStraightLine(const gtsam::Pose2& init_pose,
    const gtsam::Vector& init_conf, const gtsam::Pose2& end_pose, const gtsam::Vector& end_conf, size_t total_step);

/**
 * @brief initialize the trajectory in configuration space as a straight line
 * @param init_pose trajectory start pose
 * @param end_pose  trajectory end pose
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
GPMP2_EXPORT gtsam::Values initPose2TrajStraightLine(const gtsam::Pose2& init_pose,
    const gtsam::Pose2& end_pose, size_t total_step);

/**
 * @brief robot arm trajectory interpolator
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @return interpolated values, x(0) - x(total_step), v(0) - v(total_step)
 */
GPMP2_EXPORT gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t output_inter_step);

/**
 * @brief robot arm trajectory interpolator between any two states
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @param start_index interpolate from this state
 * @param end_index interpolate till this state
 * @return interpolated values, x(start_index) - x(end_index), v(start_index) - v(end_index)
 */
GPMP2_EXPORT gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t output_inter_step, 
    size_t start_index, size_t end_index);

/**
 * @brief mobile arm trajectory interpolator between any two states
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @param start_index interpolate from this state
 * @param end_index interpolate till this state
 * @return interpolated values, x(start_index) - x(end_index), v(start_index) - v(end_index)
 */
GPMP2_EXPORT gtsam::Values interpolatePose2MobileArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);

/**
 * @brief mobile base trajectory interpolator between any two states
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @param start_index interpolate from this state
 * @param end_index interpolate till this state
 * @return interpolated values, x(start_index) - x(end_index), v(start_index) - v(end_index)
 */
GPMP2_EXPORT gtsam::Values interpolatePose2Traj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);
}
