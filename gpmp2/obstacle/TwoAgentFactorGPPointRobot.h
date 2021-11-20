
#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/obstacle/TwoAgentFactorGP.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef TwoAgentFactorGP<PointRobotModel, GaussianProcessInterpolatorLinear> TwoAgentFactorGPPointRobot;

}
