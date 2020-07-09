% @author Mark Finean 
% @date June 10, 2020

close all;
clear all;
clc;
% profile on

import gtsam.*
import gpmp2.*
%% Setup
env = loadPredefinedMovingEnvironment('PandaEnvOnePillar', 150, 0.02, [-1,-1,-1]);

dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);



%% arm model
arm = generateArm('Panda', Pose3(Rot3(eye(3)), Point3([0,-0.1,-0.5]')));
arm_model = arm.fk_model();

% Problem setup
start_conf =  [1.57,           0.185,   0,       -1.70,   0,        3.14,   0]';
end_conf = [0,           0.185,   0,       -1.70,   0,        3.14,   0]';
% end_conf = [0,          -0.785,   0,       -2.356,   0,        1.57,   0]';
% end_conf = [0,          -0.785,   0,       -2.356,   0,        1.57,   0]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);


% plot problem setting
figure(1); hold on;
title('3D Environment')
grid on, view(3)
axis([-1 1.5 -1.2 1.5 -1 2]);
xlabel('x');
ylabel('y');
zlabel('z');
plot3DEnvironment(dataset, X, Y, Z)
hold on;
% gpmp2.plotRobotModel(arm, start_conf)
plotArm(arm.fk_model(), start_conf, 'b', 2)
plotArm(arm.fk_model(), end_conf, 'r', 2)
pause(1);

%%
% Setup

total_time_sec = 3.0;
delta_t = 0.1;
interp_multiplier = 20;
cost_sigma = 0.05;
%     cost_sigma = 0.2;
epsilon_dist = 0.2;    
limit_v = false;
limit_x = false;


% Planner settings
total_time_step = round(total_time_sec/delta_t);

problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v);


% get datasets
datasets = [];

for i = 0:total_time_step
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   

%% Solve for each scenario
% 
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

static_case = staticUpdateCase(datasets, init_values, problem_setup);
full_case = fullKnowledgeUpdateCase(datasets, init_values, problem_setup);
update_case = updateCase(datasets, init_values, problem_setup);




%%
traj = full_case.final_result;

lab_axis_lims = [-1 1.5 -1.2 1.5 -1 2];

clear_frames = true;
plot_env = true;

[X, Y, Z] = getEnvironmentMesh(datasets(1));
figure(2); hold on;
set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');

for i = 0:problem_setup.total_time_step
    key_pos = gtsam.symbol('x', i);
    conf = traj.atVector(key_pos);
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, problem_setup.arm, datasets(i+1).sdf, problem_setup.cost_sigma, ...
        problem_setup.epsilon_dist);

    if clear_frames
        cla;
    end

    if plot_env
        if i > 0
            delete(h1);
        end
        h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
    end
    
    gpmp2.plotRobotModel(problem_setup.arm, conf);

%     if any(obs_factor.spheresInCollision(conf))
%         static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
%     else
%         static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
%     end

    pause(0.2);
end

