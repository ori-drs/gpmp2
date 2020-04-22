% 
% @author Mark Finean 
% @date April 13, 2020

close all;
clear all;
clc;
% profile on

import gtsam.*
import gpmp2.*


%% Setup
t_start_moving = 0;
v_or_t_end = true;
v_or_t_end_value = [0,-0.08, 0];
starting_pos = [0.40, 0.2, 0.4];
obs_size = [0.2, 0.2, 0.2];


%% Create the environment
env = movingEnvironment3D();
env.add_object(0,...
                v_or_t_end, ...
                v_or_t_end_value, ...
                starting_pos, ...
                obs_size);
   
dataset = env.queryEnv(0);
dataset2 = env.queryEnv(1);
dataset3 = env.queryEnv(2);

[X, Y, Z] = getEnvironmentMesh(dataset);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

% algo settings
cost_sigma = 0.1;
epsilon_dist = 0.1;

graph = gtsam.NonlinearFactorGraph;
init_values = gtsam.Values;

for i = 0:500
    key_pos1 = gtsam.symbol('x', i);

    fact = gpmp2.ObstacleSDFFactorArm(key_pos1, ...
                                            arm, ...
                                            dataset.sdf, ...
                                            cost_sigma, ...
                                            epsilon_dist); 
    graph.add(fact);                                  
end

tic;
fact.replaceSDFData(dataset3.sdf);
toc;

tic;      
graph.at(250).replaceSDFData(dataset2.sdf);
toc


tic;  
fact = gpmp2.ObstacleSDFFactorArm(key_pos1, ...
                                        arm, ...
                                        dataset3.sdf, ...
                                        cost_sigma, ...
                                        epsilon_dist); 
                                        
graph.replace(250, fact);
toc
disp("it worked");
