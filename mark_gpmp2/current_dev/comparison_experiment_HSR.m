
% @author Mark Finean 
% @date June 01, 2020

close all;                                                            
clear all;
clc;
    
import gpmp2.*
import gtsam.*

% Run the lab experiments
env_size = 96;
res = 0.04;
speeds = 0:0.1:3.0;

base_pos = [0, 0, 0.4];

config_names = ["ready", "right_ready", "pickup_right", "pickup_left", ... % Start confs
                "table_forward", "table_right", "table_left", ... % Goal confs
                "table_forward_horz", "table_left_horz", "table_right_horz"]; % Goal confs

conf_combo = [
              1,5;1,6;1,7;1,8;1,9;1,10;...
              2,5;2,6;2,7;2,8;2,9;2,10;...
              3,5;3,6;3,7;3,8;3,9;3,10;...
              4,5;4,6;4,7;4,8;4,9;4,10];

conf_combo = [conf_combo; fliplr(conf_combo)]; % Include reverse as well

num_exps = size(conf_combo,1) * size(speeds,2);

results_log = zeros(num_exps, 8); % 8 fields

exp_num = 1;
for i = 1: size(conf_combo,1)
    start_conf = setPandaConf(config_names(conf_combo(i, 1))) ;
    end_conf = setPandaConf(config_names(conf_combo(i, 2))) ;
    
    for j = 1:size(speeds,2)
        
        speed = speeds(j);
        
        disp("Experiment " + num2str(exp_num) + "/" + num2str(num_exps));
        all_cases = runHSRExperiment(env_size, res, start_conf,  end_conf, speed);

        results_log(exp_num,:) =  [1, ...
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

%% Save results
% writematrix(results_log,'/home/mark/installs/gpmp2/mark_gpmp2/current_dev/hsr_results.csv')

                    