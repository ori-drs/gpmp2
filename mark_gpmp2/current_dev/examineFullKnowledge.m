
% @author Mark Finean 
% @date June 01, 2020

close all;
clear all;
clc;
    
import gpmp2.*
import gtsam.*

% Run the lab experiments
env_size = 75;
res = 0.04;
speed = 0.6;

start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';



lab_axis_lims = [-1 1.5 -1.2 1.5 -1 2];


total_time_sec = 3.0;
delta_t = 0.1;
interp_multiplier = 20;
cost_sigma = 0.05;
epsilon_dist = 0.3;    
limit_v = false;
limit_x = false;

env = loadPaperEnvs(1, env_size, res, speed);
% 
% 
%% Planner settings

total_time_step = round(total_time_sec/delta_t);

problem_setup = paperGetProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v);

%% get datasets
disp('Getting all the datasets');
datasets = [];

for i = 0:total_time_step
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   

[X, Y, Z] = getEnvironmentMesh(datasets(1));

%% Solve for each scenario
import gpmp2.*
import gtsam.*

init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

disp('Case2: Full knowledge sdf');
full_knowledge_case = manualFullKnowledgeCase(datasets, init_values, problem_setup);

%% Check fol collisions
% 
% full_knowledge_confs_in_collision = 0;
% for i = 0:problem_setup.total_time_step
%     key_pos = gtsam.symbol('x', i);
%     obs_factor = gpmp2.ObstacleSDFFactorArm(...
%                  key_pos, problem_setup.arm, datasets(i+1).sdf, ...
%                  cost_sigma, epsilon_dist);
% 
%     full_knowledge_conf = full_knowledge_case.final_result.atVector(key_pos);
% 
%     if any(obs_factor.spheresInCollision(full_knowledge_conf))
%         full_knowledge_confs_in_collision = full_knowledge_confs_in_collision + 1;
%     end
% end
% 
% 
% 
% rotate3d on
% figure(1); hold on; 
% set(gcf,'Position',[1350 500 1200 1400]);
% title('3D Environment')
% grid on;
% view(3);
% axis(lab_axis_lims);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3DEnvironment(datasets(1), X, Y, Z)
% gpmp2.plotArm(problem_setup.arm.fk_model(), ...
%         problem_setup.start_conf, 'b', 2)
% gpmp2.plotArm(problem_setup.arm.fk_model(), ...
%         problem_setup.end_conf, 'r', 2)






















