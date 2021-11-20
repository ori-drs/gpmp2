
#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/obstacle/AgentAvoidanceFactorGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef AgentAvoidanceFactorGP<PointRobotModel, GaussianProcessInterpolatorLinear> AgentAvoidanceFactorGPPointRobot;

}
