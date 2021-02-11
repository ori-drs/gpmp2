clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
plot_graphs = false;
total_time_sec = 5;
delta_t = 0.2;
interp_multiplier = 10;
cost_sigma = 0.05;
epsilon_dist = 0.1;    
limit_v = false;
limit_x = false;
cell_size = 0.04;
env_size = 64;
origin = [-1.28,-1.28,-1.28];
base_pos = [0, 0, 0];
total_time_step = round(total_time_sec/delta_t);


%% Environment

% Obstacle def
starting_pos1 = [0.35, -0.35, 0.35];
obs_size = [0.3, 0.3, 0.2];

% Create the environment
env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object(starting_pos1, obs_size);
env.updateMap();

% dataset = env.queryEnv();
% sdf = dataset.sdf;
use_signed = false;
if use_signed
    sdf = env.getSDF();
else
    sdf = env.getUSDF();
end

%% Task

% Setup problem
start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');


problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v, base_pos);             


%% Planning
init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);

panda_planner = pandaGraphMaintainer(sdf, problem_setup);

[result, error, iterations] = panda_planner.optimize(init_values);

results_log = zeros(40000, 6); % [index, signed(1)/unsigned(0) error, collisions, iterations]
results_log(1, :) = double([1, 1, 1, error, double(num_collisions), double(iterations)]);

%% Analysis

% How many collisions?
collision_timeline = zeros(total_time_step + 1);
for t = 0:total_time_step
    key_pos = gtsam.symbol('x', t);
    conf = result.atVector(key_pos);
    collision_timeline(t+1) = panda_planner.collisionCheck(conf);
    num_collisions = sum(collision_timeline, 'all');
end

disp("Error: " + num2str(error) + sprintf('\t') + "Iterations: " + num2str(iterations) +  sprintf('\t')  + "Collisions: " + num2str(num_collisions));
% How many iterations?
% Total error
% Time taken to plan



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

key_pos = gtsam.symbol('x', t);
conf = result.atVector(key_pos);

h1 = plot3DEnvironment(dataset, X, Y, Z);

static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);

pause(0.05);
end



% 
% obs_factor = gpmp2.ObstacleSDFFactorArm(...
%     key_pos, problem_setup.arm, sdf, problem_setup.cost_sigma, ...
%     problem_setup.epsilon_dist);

%         if any(obs_factor.spheresInCollision(conf))
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
%         else
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
%         end

