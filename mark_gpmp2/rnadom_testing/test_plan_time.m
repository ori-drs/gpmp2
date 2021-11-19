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
cell_size = 0.02;
env_size = 256;
origin = [-2.56,-2.56,-2.56];
total_time_step = round(total_time_sec/delta_t);


%% Environment

% Create the environment
env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object([0.5,-1,0.75], [0.5,1,1.5]);

env.add_object([0.5,1,0.375], [0.5,1,0.75]);
env.add_object([0.5,1,1.375], [0.5,1,0.75]);

env.updateMap();

%% Task

% Setup problem
start_pos = gtsam.Pose2(-1, 0, pi/2);
start_conf = [0.4,-1,0.3,0,0]';
end_pos = gtsam.Pose2(1, 0, pi/2);
% end_conf = [0,0,0,-1.57,0]';
end_conf = [0.4,-1,0.3,0,0]';


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
tic();
[usdf_result, ~, usdf_iterations] = hsr_planner_usdf.optimize(init_values);
toc();
