/**
 *  @file  DynamicObstacleSDFFactorArm.h
 *  @brief Obstacle avoidance cost factor, using Arm and signed distance field
 *  @author Mark Finean
 *  @date  April 21, 2020
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/DynamicObstacleSDFFactor.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef DynamicObstacleSDFFactor<ArmModel> DynamicObstacleSDFFactorArm;

}


