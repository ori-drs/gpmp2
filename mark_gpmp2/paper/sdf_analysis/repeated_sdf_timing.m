
clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

cell_size = 0.02;
env_size = 96;
total_time_step = 30;
delta_t = 0.1;
epsilon = 0.2;
workspace_size = [env_size,env_size,env_size];

arm = generateArm('WAMArm');
arm_model = arm.fk_model();


% Avg data stats

env_list = ["MovingReplannerOneBlock","MovingReplanner","MovingReplannerSmallBlocks","MovingReplannerNoStatic","MovingBlock"];
% env_list = ["MovingBlock"];
num_envs = length(env_list);
num_repeats = 10;

static_field_times = zeros(num_envs, 435, num_repeats);
calc_obj_sdfs_time_times = zeros(num_envs, 435, num_repeats);
tracking_times = zeros(num_envs, 435, num_repeats);
my_times = zeros(num_envs, 435, num_repeats);
bench_times = zeros(num_envs, 435, num_repeats);

improvement = [];



for j = 1:length(env_list)
    env_name = env_list(j);
    env = loadSDFAnalysisEnvironment(env_name, env_size, cell_size);
    dataset = env.queryEnv(0);
    
    origin = [dataset.origin_x, ...
            dataset.origin_y, ...
            dataset.origin_z];

    origin_point3 = dataset.origin_point3;

    for k = 1:num_repeats
 
        object_predictor = objectTrackerPredictor(workspace_size, dataset.static_map,...
                                epsilon, cell_size, origin_point3);
        object_predictor.update(0.2, env.queryEnv(0.2).map); 
        object_predictor.update(0.6, env.queryEnv(0.6).map); 

        counter = 1;
        for t = 2:total_time_step % Don't test 0 since no vel to predict
%             object_predictor.update( t * delta_t, env.queryEnv( t * delta_t).map); 
            
            for i = t:total_time_step
                
                test_time = i * delta_t;
                                
                % Record the times taken
                static_field_times(j, counter, k) = object_predictor.init_static_field_time;
                calc_obj_sdfs_time_times(j, counter, k) = object_predictor.calc_obj_sdfs_time;
                tracking_times(j, counter, k) = object_predictor.sdf_coord_track_time;


                % Record time to predict composite sdf
                rec_t = tic;
                test_predicted_sdf = object_predictor.predict_object_locations(test_time);
                my_time = toc(rec_t);
                my_times(j, counter, k) = my_time;

                % Record time to predict exact sdf
                rec_t = tic;
                bench_predicted_sdf = object_predictor.predict_field(test_time);
                bench_time = toc(rec_t);
                bench_times(j, counter, k) = bench_time;

                improvement(end+1) = 100*(1-my_time/bench_time);
                if improvement(end) < 70
                    disp(env_name + "   my_time: " + num2str(my_time) + ...
                        "   iter: " + num2str(k)  + "   t: " + num2str(t)  + ...
                        "   i: " + num2str(i) );
                    disp('');
                end    
                counter = counter + 1;

            end
        end
        clear object_predictor;
    end
end

%%
counter = 1;
for t = 2:total_time_step
    for i = t:total_time_step
        counter = counter+1;
    end
end
                                
% figure(2); hold on;
% h1 = bar(bench_times*1000, 'FaceColor',[0, 0.4470, 0.7410]);
% h3 = bar((static_field_times + calc_obj_sdfs_time_times)*1000, 'FaceColor', [0.6350 0.0780 0.1840]);
% h2 = bar(my_times*1000, 'FaceColor', [0.4660, 0.6740, 0.1880]);
% ylim([0,700]);
% xlabel('Trial number', "FontSize", 16);
% ylabel('Time (ms)', "FontSize", 16);
% title('SDF Prediction Method Comparison', 'FontSize', 16);
% hLg = legend([h1, h2, h3], ...
%     ["Full SDF Prediction", "Approximate Method", "Initial Times"],...
%     'Location','southoutside',...
%     'NumColumns', 3, ...
%     'FontSize', 16);

% We want, the min, max, average + initial cosat
bench_stats = getStats(bench_times);
init_stats = getStats(static_field_times + calc_obj_sdfs_time_times);
prediction_stats = getStats(my_times);

Dataset = {'SDF Benchmark';'Initialisation';'Future Predictions'};
Min = [bench_stats.min; init_stats.min; prediction_stats.min];
Median = [bench_stats.median; init_stats.median; prediction_stats.median];
Mean = [bench_stats.mean; init_stats.mean; prediction_stats.mean];
Max = [bench_stats.max; init_stats.max; prediction_stats.max];

T = table(Dataset,Min,Median,Mean,Max)



% Save results
% writetable(T,"/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_timing_table_96env")
% save('/home/mark/installs/gpmp2/mark_gpmp2/paper/sdf_analysis/data/sdf_timing_results_96env','bench_stats','init_stats', 'prediction_stats')


%%
figure(2);
env.queryEnv(0.4);
[X, Y, Z] = getEnvironmentMesh(env.getDataset);
title("Graph");
xlabel('x'); ylabel('y'); zlabel('z'); view(3);
ax_lims = [-1 1.5 -1.2 1.5 -1 2];
axis(ax_lims);
h1 = plot3DEnvironment(env.getDataset, X, Y, Z);
% 




%%
function stats = getStats(data)

    stats.min = min(min(min(data)));
    stats.mean = mean(mean(mean(data)));
    stats.median = median(median(median(data)));
    stats.max = max(max(max(data)));
    
end










