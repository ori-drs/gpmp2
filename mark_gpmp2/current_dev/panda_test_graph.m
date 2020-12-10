clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

plot_graphs = false;
total_time_sec = 3;
delta_t = 1;
interp_multiplier = 1;
cost_sigma = 0.05;
epsilon_dist = 0.3;    
limit_v = false;
limit_x = false;

cell_size = 0.04;
env_size = 64;
origin = [-1,-1,-1];

base_pos = [0, 0, 0];

env = loadPredefinedMovingEnvironment('Empty', env_size, cell_size, origin);

dataset = env.queryEnv(0);
start_sdf = dataset.sdf;
    
% Setup problem
start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');


total_time_step = round(total_time_sec/delta_t);
problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v, base_pos);             

init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);

panda_planner = pandaPlanner(start_sdf, problem_setup);

t_s = tic;
result = panda_planner.optimize(init_values);
t_f = toc(t_s);

disp(t_f);
