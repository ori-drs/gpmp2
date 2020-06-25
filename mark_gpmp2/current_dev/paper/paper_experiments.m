
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
speeds = 0.1:0.1:1.0;
num_rands = 1;
setArmSeed(10);
env_num = 4 ;% 1
full_info = false;

configs = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2; ... % original start
            -0.4,-1.70,1.64,1.45,1.1,-0.206,2.2;
            0.0,0.94,0,1.6,0,-0.919,1.55;% original end
            0.0,0.6,0,0.9,0,-0.119,1.55;
            -0.4,0.94,0,1.6,-0.3,-0.919,1.55]'; 
        
% conf_combo = nchoosek(1:num_configs,2);
conf_combo = [1,3;1,4;1,5;2,3;2,4;2,5;3,4];

num_speeds = size(speeds,2);
num_combos = size(conf_combo,1);
num_exps = size(conf_combo,1) * size(speeds,2) * num_rands;

results_log = zeros(num_exps, 8);

exp_num = 1;
for i = 1:num_combos
    start_conf = configs(:, conf_combo(i,1));
    end_conf = configs(:, conf_combo(i,2));
    
    for j = 1:num_speeds
        
        speed = speeds(j);
        
        for k = 1:num_rands % random trials
            disp("Experiment " + num2str(exp_num) + "/" + num2str(num_exps));
            all_cases = runPaperExperiment(env_size, res, start_conf,  end_conf, env_num, speed, full_info);
%             results_log{i, j, k} = all_cases;
            results_log(exp_num,:) =  [i, ...
                                speed, ...
                                all_cases.static_case.num_collisions, ...
                                all_cases.execute_update_case.num_collisions,...
                                all_cases.full_knowledge_case.num_collisions,...
                                all_cases.static_case.actual_cost, ...
                                all_cases.execute_update_case.actual_cost,...
                                all_cases.full_knowledge_case.actual_cost];
            exp_num = exp_num + 1;
        end
        
    end
    
end


%%
start_conf = configs(:, conf_combo(7,1));
end_conf = configs(:, conf_combo(7,2));
speed = 0.4;

all_cases = runPaperExperiment(env_size, res, start_conf,  end_conf, env_num, speed, true);

lab_axis_lims = [-1 1.5 -1.2 1.5 -1 2];
figure(1); hold on;
plotRobotModelTrajectory(all_cases.full_knowledge_case, ...
                        all_cases.datasets, ...
                        all_cases.problem_setup, ...
                        lab_axis_lims, 0.1)