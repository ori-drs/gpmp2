
#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/TwoAgentFactor.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef TwoAgentFactor<PointRobotModel> TwoAgentFactorPointRobot;

}
