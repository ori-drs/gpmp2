clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
plot_graphs = false;
total_time_sec = 2;
delta_t = 0.2;
interp_multiplier = 2;
cost_sigma = 0.01;
epsilon_dist = 0.1;  

cell_size = 0.01;
env_size = 500;
origin = [-5,-5];
total_time_step = round(total_time_sec/delta_t);

use_signed = false;

%% Environment

% Obstacle def
starting_pos1 = [-2.5, -2.5];
obs_size = [1, 3];
% obs_size = [3,1];

% Create the environment
env = load2DComparisonEnvironment(env_size, cell_size, origin);
env.add_object([-5,-2.5], [5,0.4]);
env.add_object([0,-2.5], [5,0.4]);
env.add_object([-2.5,-5], [0.4,5]);
env.add_object([-2.5, 0], [0.4,5]);

env.add_object(starting_pos1, obs_size);
   
    
env.updateMap();

if use_signed
    sdf = env.getSDF();
else
    sdf = env.getUSDF();
end

%% Task
% Setup problem
start_pose = gtsam.Pose2(-4, -4, pi/2);
end_pose = gtsam.Pose2(-2, -1, pi/2);

problem_setup = mobileProblemSetup2D(start_pose, end_pose, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier);
                                 
%% Planning
init_values = init2DMobileValues(problem_setup);

planner = graphMaintainer2D(sdf, problem_setup);

[result, error, iterations] = planner.optimize(init_values);


%% Analysis

% How many collisions?
collision_timeline = zeros(total_time_step + 1);
for t = 0:total_time_step
    key_pos = gtsam.symbol('x', t);
    conf = result.atPose2(key_pos);
    collision_timeline(t+1) = planner.collisionCheck(conf);
    num_collisions = sum(collision_timeline, 'all');
end

disp("Error: " + num2str(error) + sprintf('\t') + "Iterations: " + num2str(iterations) +  sprintf('\t')  + "Collisions: " + num2str(num_collisions));

%% Visualisations

dataset = env.queryEnv();
plot_inter = 0;
if plot_inter
    total_plot_step = total_time_step * (plot_inter + 1);
    plot_values = gpmp2.interpolatePose2Traj(result, problem_setup.Qc_model, problem_setup.delta_t, plot_inter, 0, problem_setup.total_time_step);
else
    total_plot_step = total_time_step;
    plot_values = result;
end
figure(4), hold on
plotEvidenceMap2D(permute(dataset.map, [2,1]), dataset.origin_x, dataset.origin_y, cell_size);
for i=0:total_plot_step
    p = plot_values.atPose2(symbol('x', i));
    plotPlanarMobileBase(problem_setup.robot.fk_model(), p, [0.4 0.2], 'b', 1);
end
hold off;