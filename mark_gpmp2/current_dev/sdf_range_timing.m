
clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

total_time_step = 30;
delta_t = 0.1;
epsilon = 0.2;

num_repeats = 10;


% env_sizes = [32, 64, 96, 128, 160, 192, 224, 256, 288, 310];
% env_sizes = [25, 32, 50, 64, 75, 96, 100, 125, 128, 150, 160, 175, 192, 200, 224, 225, 256, 275, 288, 300, 310];
% env_sizes = [50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300];
% env_sizes = 50:10:300;
env_sizes = 64:32:320;
num_env_sizes = length(env_sizes);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

env_list = ["MovingReplannerOneBlock","MovingReplanner","MovingReplannerSmallBlocks","MovingReplannerNoStatic","MovingBlock"];
% env_list = ["MovingReplannerOneBlock"];
num_envs = length(env_list);

%% Avg data stats
bench_results = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);
prediction_results = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);
init_results = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);
size_array = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);

num_vals = (total_time_step + 1) * num_envs;
for k = 1:num_env_sizes
    env_size = env_sizes(k);
    disp('Size:');
    disp(env_size);
    
    cell_size = ceil(300/env_size)/100;
    
    workspace_size = [env_size,env_size,env_size];

    for p = 1:num_repeats
        static_field_times = zeros(num_envs, total_time_step + 1);
        calc_obj_sdfs_time_times = zeros(num_envs, total_time_step + 1);
        tracking_times = zeros(num_envs, total_time_step + 1);
        my_times = zeros(num_envs, total_time_step + 1);
        bench_times = zeros(num_envs, total_time_step + 1);  
        
        for j = 1:length(env_list)
        
            env_name = env_list(j);
            env = loadSDFAnalysisEnvironment(env_name, env_size, cell_size);
            dataset = env.queryEnv(0);

            origin = [dataset.origin_x, ...
                    dataset.origin_y, ...
                    dataset.origin_z];

            origin_point3 = dataset.origin_point3;

            object_predictor = objectTrackerPredictor(workspace_size, dataset.static_map,...
                                                    epsilon, cell_size, origin_point3);

            % Give two initial measurements
            object_predictor.update(0.2, env.queryEnv(0.2).map); 
            object_predictor.update(0.6, env.queryEnv(0.6).map); 
                
            for i = 0:total_time_step
                test_time = i * delta_t;

                % Record the times taken
                static_field_times(j, i+1) = object_predictor.init_static_field_time;
                calc_obj_sdfs_time_times(j, i+1) = object_predictor.calc_obj_sdfs_time;
                tracking_times(j, i+1) = object_predictor.sdf_coord_track_time;


                % Record time to predict composite sdf
                tic;
                test_predicted_sdf = object_predictor.predict_object_locations(test_time);
                my_time = toc;
                my_times(j, i+1) = my_time;

                % Record time to predict exact sdf
                tic;
                bench_predicted_sdf = object_predictor.predict_field(test_time);
                bench_time = toc;
                bench_times(j, i+1) = bench_time;

            end
            
            clear object_predictor
  
        end
        
        bench_times = reshape(bench_times,1,[])';
        my_times = reshape(my_times,1,[])';
        static_field_times = reshape(static_field_times,1,[])';
        calc_obj_sdfs_time_times = reshape(calc_obj_sdfs_time_times,1,[])';

        bench_results((1:num_vals)+ (k+p-2)*num_vals,1) = bench_times;
        prediction_results((1:num_vals)+ (k+p-2)*num_vals,1) = my_times;
        init_results((1:num_vals)+ (k+p-2)*num_vals,1) = static_field_times + calc_obj_sdfs_time_times;
        size_array((1:num_vals)+ (k+p-2)*num_vals,1) = env_size; 
        
    end


end
%%
% save('/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_range_timing_results_allenvs_rep10','bench_results','prediction_results', 'init_results', 'size_array')

load('/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_range_timing_results_allenvs_rep10')

figure(2);
title('SDF Computation Time')
xlabel('Size')
ylabel('Time (ms)')
% boxplot(bench_results*1000,size_array);
hold on;
boxplot(bench_results*1000,size_array);


init_group_mean = grpstats(init_results*1000, size_array, @mean);
init_group_std = grpstats(init_results*1000, size_array, @std);

bench_group_mean = grpstats(bench_results*1000, size_array, @mean);
bench_group_std = grpstats(bench_results*1000, size_array, @std);

prediction_group_mean = grpstats(prediction_results*1000, size_array, @mean);
prediction_group_std = grpstats(prediction_results*1000, size_array, @std);

bench_ratio_error = bench_group_std./bench_group_mean;
prediction_ratio_error = prediction_group_std./prediction_group_mean;

improvement = 100*(1 - (prediction_group_mean./bench_group_mean));

figure(3);
title('SDF Computation Time')
xlabel('Size')
ylabel('Time (ms)')
hold on;
% plot(log(env_sizes), log(bench_group_mean));
% plot(log(env_sizes), log(prediction_group_mean));

errorbar(env_sizes, bench_group_mean, bench_group_std);
errorbar(env_sizes, prediction_group_mean, prediction_group_std);
xticks(env_sizes);


set(gca, 'YScale', 'log');


table_names = {'Workspace Size', 'Mean', 'Std'};


T = table(env_sizes',bench_group_mean,bench_group_std,prediction_group_mean,prediction_group_std)





