clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
total_time_sec = 5;
delta_t = 0.2;
interp_multiplier = 0;

cost_sigma = 0.05;
epsilon_dist = 0.1;    
limit_v = false;
limit_x = false;
cell_size = 0.04;
env_size = 64;
origin = [-1.28,-1.28,-1.28];
total_time_step = round(total_time_sec/delta_t);


%% Environment

% Create the environment
env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object([1,1,1], [1,1,1]);
env.updateMap();

%% Task

% Setup problem
start_pos = gtsam.Pose2(0, 0, 0);
start_conf = [0,0,0,0,0]';
end_pos = gtsam.Pose2(0, 0, 0);
end_conf = [0,0,0,-1.57,0]';


problem_setup = HSRProblemSetup(gpmp2.Pose2Vector(start_pos, start_conf), gpmp2.Pose2Vector(end_pos, end_conf), total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v);             

evaluation_setup = HSRProblemSetup(gpmp2.Pose2Vector(start_pos, start_conf), gpmp2.Pose2Vector(end_pos, end_conf), total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, 20, ...
                                     limit_x, limit_v);   

init_values = gpmp2.initPose2VectorTrajStraightLine(start_pos, start_conf, end_pos, end_conf, total_time_step);

%% Planning

usdf = env.getUSDF();
hsr_planner_usdf = HSRGraphMaintainer(usdf, problem_setup);
[usdf_result, ~, usdf_iterations] = hsr_planner_usdf.optimize(init_values);

sdf = env.getSDF();
hsr_planner_sdf = HSRGraphMaintainer(sdf, problem_setup);
[sdf_result, ~, sdf_iterations] = hsr_planner_sdf.optimize(init_values);

evaluation_planner = HSRGraphMaintainer(sdf, evaluation_setup);

sdf_error = evaluation_planner.error(sdf_result);
usdf_error = evaluation_planner.error(usdf_result);

%% Analysis

% How many collisions?
sdf_collision_timeline = zeros(total_time_step + 1);
for t = 0:total_time_step
    key_pos = gtsam.symbol('x', t);
    sdf_conf = gpmp2.atPose2VectorValues(key_pos, sdf_result);
    sdf_collision_timeline(t+1) = evaluation_planner.collisionCheck(sdf_conf);
    sdf_num_collisions = sum(sdf_collision_timeline, 'all'); 
end

usdf_collision_timeline = zeros(total_time_step + 1);
for t = 0:total_time_step
    key_pos = gtsam.symbol('x', t);
    usdf_conf = gpmp2.atPose2VectorValues(key_pos, usdf_result);
    usdf_collision_timeline(t+1) = evaluation_planner.collisionCheck(usdf_conf);
    usdf_num_collisions = sum(usdf_collision_timeline, 'all'); 
end


%% Visualisations

% Plot simulation

dataset = env.queryEnv();
[X, Y, Z] = getEnvironmentMesh(dataset);
figure(1); hold on; cla;
lab_axis_lims = [-1.5 1.5 -1.5 1.5 -0.5 1];
set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); 
grid on; 
view(3);
xlabel('x'); ylabel('y'); zlabel('z');

for t = 0:total_time_step
cla;
h1 = plot3DEnvironment(dataset.map, X, Y, Z);
% 
key_pos = gtsam.symbol('x', t);
pose = gpmp2.atPose2VectorValues(key_pos, usdf_result);

h1 = plot3DEnvironment(dataset, X, Y, Z);

static_handle = gpmp2.plotRobotModel(problem_setup.arm, pose);

pause(0.05);
end

% 
% 
% % 
% % obs_factor = gpmp2.ObstacleSDFFactorArm(...
% %     key_pos, problem_setup.arm, sdf, problem_setup.cost_sigma, ...
% %     problem_setup.epsilon_dist);
% 
% %         if any(obs_factor.spheresInCollision(conf))
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
% %         else
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
% %         end
% 

