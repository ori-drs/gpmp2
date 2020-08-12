
clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

cell_size = 0.01;
env_size = 300;

env_name = 'MovingReplanner';
env = loadPredefinedMovingEnvironment(env_name, env_size, cell_size);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

total_time_step = 30;
delta_t = 0.5;

dataset = env.queryEnv(0);

origin = [dataset.origin_x, ...
        dataset.origin_y, ...
        dataset.origin_z];
    
epsilon = 0.2;
origin_point3 = dataset.origin_point3;
workspace_size = size(dataset.map);

%% Timing normal

tic; field  = gpmp2.signedDistanceField3D(dataset.map, 0.01); toc;

%% Single stats
object_predictor = objectTrackerPredictor(workspace_size, dataset.static_map,...
                                            epsilon, cell_size, origin_point3);

% for i = 0:total_time_step
% profile on
% 
% for i = 0:1
%     disp("Update SDF step: " + num2str(i));
%     
% %     new_map = env.queryEnv(i*delta_t).map - flip(env.queryEnv(i*problem_setup.delta_t).static_map);
% %     [X, Y, Z] = getEnvironmentMesh(env.queryEnv(i*delta_t));
% %     figure(1); hold on;
% %     plot3DEnvironment(new_map)
% 
%     object_predictor.update(i*delta_t, env.queryEnv(i*delta_t).map); 
% %     tic;
%     predicted_sdf = object_predictor.predict_object_locations((i+1)*delta_t);
%     profile viewer
% %     toc;
% end

% Give two initial measurements
object_predictor.update(0.2, env.queryEnv(0.2).map); 
object_predictor.update(0.6, env.queryEnv(0.6).map); 

disp("Static field init time: " + num2str(object_predictor.init_static_field_time));
disp("Time to calc two sdf objects: " + num2str(object_predictor.calc_obj_sdfs_time));
disp("Tracking time: " + num2str(object_predictor.sdf_coord_track_time));


format short;

test_time = delta_t;

tic;
% profile on;
test_predicted_sdf = object_predictor.predict_object_locations(test_time);
% profile viewer;
my_time = toc;
disp("Min method predict time " + num2str(my_time));

% test_predicted_sdf(test_predicted_sdf>2) = 2;

% 
tic;
bench_predicted_sdf = object_predictor.predict_sdf(test_time);
benc_time = toc;
disp("Standard predict time " + num2str(benc_time));



disp("Time reduced by " + num2str(100*(1- my_time/benc_time)) + "%");
% Static field init time: 0.60855
% Time to calc two sdf objects: 0.054638
% Total SDF init time = 0.6632

% Tracking time: 0.000541
% Time reduced by 83.7588%
% Min method predict time 0.07646
% Standard predict time 0.66963
% Add on 26ms to permute for both


% @ 2cm resolution
% 3ms to permute
% Static field init time: 0.10701
% Time to calc two sdf objects: 0.014181
% Tracking time: 0.000539
% Min method predict time 0.009887
% Standard predict time 0.093974
% Time reduced by 89.479%

%% Avg data stats

static_field_times = zeros(1, total_time_step + 1);
calc_obj_sdfs_time_times = zeros(1, total_time_step + 1);
tracking_times = zeros(1, total_time_step + 1);
my_times = zeros(1, total_time_step + 1);
bench_times = zeros(1, total_time_step + 1);

object_predictor = objectTrackerPredictor(workspace_size, dataset.static_map,...
                                            epsilon, cell_size, origin_point3);

for i = 0:total_time_step
    test_time = i * delta_t;

    object_predictor = objectTrackerPredictor(workspace_size, dataset.static_map,...
                                            epsilon, cell_size, origin_point3);
                                        
    % Give two initial measurements
    object_predictor.update(0.2, env.queryEnv(0.2).map); 
    object_predictor.update(0.6, env.queryEnv(0.6).map); 
    static_field_times(i+1) = object_predictor.init_static_field_time;
    calc_obj_sdfs_time_times(i+1) = object_predictor.calc_obj_sdfs_time;
    tracking_times(i+1) = object_predictor.sdf_coord_track_time;



    tic;
    test_predicted_sdf = object_predictor.predict_object_locations(test_time);
    my_time = toc;
    my_times(i+1) = my_time;
    
    tic;
    bench_predicted_sdf = object_predictor.predict_sdf(test_time);
    bench_time = toc;
    bench_times(i+1) = bench_time;
    
    clear object_predictor
end
%%
% figure(1); hold on;
% subplot(1,2,1); hold on;
% bar(my_times*1000);
% ylim([0,120]);
% 
% subplot(1,2,2); hold on;
% bar(bench_times*1000);
% ylim([0,120]);

figure(2); hold on;
h1 = bar(bench_times*1000, 'FaceColor',[0, 0.4470, 0.7410]);
h3 = bar((static_field_times + calc_obj_sdfs_time_times)*1000, 'FaceColor', [0.6350 0.0780 0.1840]);
h2 = bar(my_times*1000, 'FaceColor', [0.4660, 0.6740, 0.1880]);
ylim([0,700]);
xlabel('Trial number', "FontSize", 16);
ylabel('Time (ms)', "FontSize", 16);
title('SDF Prediction Method Comparison', 'FontSize', 16);
hLg = legend([h1, h2, h3], ...
    ["Full SDF Prediction", "Approximate Method", "Initial Times"],...
    'Location','southoutside',...
    'NumColumns', 3, ...
    'FontSize', 16);

% We want, the min, max, average + initial cosat
bench_stats = getStats(bench_times);
init_stats = getStats(static_field_times + calc_obj_sdfs_time_times);
prediction_stats = getStats(my_times);


function stats = getStats(data)

    stats.min = min(data);
    stats.mean = mean(data);
    stats.median = median(data);
    stats.max = max(data);
    
end

























