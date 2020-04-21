/**
 *  @file  DynamicObstacleSDFFactorGPArm.h
 *  @brief Obstacle avoidance cost factor, using Arm planar, linear GP and signed distance field
 *  @author Mark Finean
 *  @date  April 21, 2020
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/obstacle/DynamicObstacleSDFFactorGP.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef DynamicObstacleSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear> 
    DynamicObstacleSDFFactorGPArm;

}


