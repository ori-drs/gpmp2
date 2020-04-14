% planar arm obstacle avoidance example with moving obstacle, build factor graph in matlab
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 18, 2020

close all;
clear all;
clc;

import gtsam.*
import gpmp2.*

t_start_moving = 0;
v_or_t_end = true;
v_or_t_end_value = [0.2,-0.2];
starting_pos = [-0.40, 1.90];
obs_size = [0.60, 0.80];

env = movingEnvironment(0,v_or_t_end,v_or_t_end_value, starting_pos, obs_size);
dataset = env.queryEnv(0);

end_conf = [pi/2, 0]';
end_vel = [0, 0]';

% arm model
arm = generateArm('SimpleTwoLinksArm');
arm_model = arm.fk_model();

costmap = vehicleCostmap(flip(dataset.map));
% planner = pathPlannerRRT(costmap, 'GoalTolerance', [5 5 5]);
planner = pathPlannerRRT(costmap);

startPose = [250, 150, 90]; % [meters, meters, degrees]
goalPose = [150, 250, 0];


refPath = plan(planner,startPose,goalPose);
plot(planner)


transitionPoses = interpolate(refPath);
hold on
plot(refPath,'DisplayName','Planned Path')
scatter(transitionPoses(:,1),transitionPoses(:,2),[],'filled', ...
    'DisplayName','Transition Poses')
hold off