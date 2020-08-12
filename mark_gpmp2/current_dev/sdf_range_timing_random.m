
clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

total_time_step = 30;
delta_t = 0.1;
epsilon = 0.2;

num_repeats = 5;
env_list = ["MovingReplannerOneBlock","MovingReplanner","MovingReplannerSmallBlocks","MovingReplannerNoStatic","MovingBlock"];

env_sizes = 50:10:300;
num_env_sizes = length(env_sizes);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

num_envs = length(env_list);

%% Avg data stats
bench_results = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);
prediction_results = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);
init_results = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);
size_array = zeros(total_time_step+1 * num_envs * num_env_sizes * num_repeats, 1);

num_vals = (total_time_step + 1) * num_envs;

improvement = [];
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

                improvement(end+1) = 100*(1-my_time/bench_time);
                
                clear object_predictor
            end
  
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

% save('/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_range_timing_results','bench_results','prediction_results', 'init_results', 'size_array')

figure(2);
title('SDF Computation Time')
xlabel('Size')
ylabel('Time (ms)')
% boxplot(bench_results*1000,size_array);
hold on;
boxplot(prediction_results*1000,size_array);

% [sorted_prediction_results,sorted_size_array] = sort(prediction_results,size_array);
figure(2);
title('SDF Computation Time')
xlabel('Size')
ylabel('Time (ms)')
boxplot(sorted_prediction_results*1000,sorted_size_array);

bench_group_mean = grpstats(bench_results*1000, size_array, @mean);
prediction_group_mean = grpstats(prediction_results*1000, size_array, @mean);

bench_group_std = grpstats(bench_results*1000, size_array, @std);
prediction_group_std = grpstats(prediction_results*1000, size_array, @std);

figure(3);
title('SDF Computation Time')
xlabel('Size')
ylabel('Time (ms)')
hold on;
% plot(log(env_sizes), log(bench_group_mean));
% plot(log(env_sizes), log(prediction_group_mean));

errorbar(env_sizes, bench_group_mean, bench_group_std);
errorbar(env_sizes, prediction_group_mean, prediction_group_std);



% We want, the min, max, average + initial cosat
% bench_stats = getStats(bench_times);
% init_stats = getStats(static_field_times + calc_obj_sdfs_time_times);
% prediction_stats = getStats(my_times);
% 
% Dataset = {'SDF Benchmark';'Initialisation';'Future Predictions'};
% Min = [bench_stats.min; init_stats.min; prediction_stats.min];
% Median = [bench_stats.median; init_stats.median; prediction_stats.median];
% Mean = [bench_stats.mean; init_stats.mean; prediction_stats.mean];
% Max = [bench_stats.max; init_stats.max; prediction_stats.max];
% 
% T = table(Dataset,Min,Median,Mean,Max)
% 
% 
% function stats = getStats(data)
% 
%     stats.min = min(min(data));
%     stats.mean = mean(mean(data));
%     stats.median = median(median(data));
%     stats.max = max(max(data));
%     
% end

% Save results
% writetable(T,"/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_timing_table")
% save('/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_timing_results','bench_stats','init_stats', 'prediction_stats')






















