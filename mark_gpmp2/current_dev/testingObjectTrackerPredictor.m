
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


object_predictor.update(0.2, env.queryEnv(0.2).map); 
object_predictor.update(0.6, env.queryEnv(0.6).map); 
% % profile on;
% tic;
% my_predicted_sdf = object_predictor.predict_object_locations(1);
% my_time = toc;
% % profile viewer;
% tic;
% bench_predicted_sdf = object_predictor.predict_sdf(1);
% bench_time = toc;
% 100*(1- my_time/bench_time)

test_time = delta_t;

tic;
% profile on 
test_predicted_sdf = object_predictor.predict_object_locations(test_time);
% profile viewer 
my_time = toc;

test_predicted_sdf(test_predicted_sdf>2) = 2;

% 
tic;
bench_predicted_sdf = object_predictor.predict_sdf(test_time);
benc_time = toc;
format short;
disp("Time reduced by " + num2str(100*(1- my_time/benc_time)) + "%");

% 
actual_sdf = env.queryEnv(test_time).field;
diff = actual_sdf - test_predicted_sdf;

figure(1); hold on;
subplot(1,3,1); hold on;
title('Predicted');
h1 = plotSignedDistanceField2D(test_predicted_sdf(:,:,150), origin(1), origin(2), cell_size, epsilon);
subplot(1,3,2); hold on;
title('Actual');
h2 = plotSignedDistanceField2D(actual_sdf(:,:,150), origin(1), origin(2), cell_size, epsilon);
subplot(1,3,3); hold on;
title('Diff');
h3 = plotSignedDistanceField2D(diff(:,:,150), origin(1), origin(2), cell_size, epsilon);







dil_env = loadPredefinedMovingEnvironment("DilatedMovingObjects", env_size, cell_size);
no_stat_env = loadPredefinedMovingEnvironment("MovingReplannerNoStatic", env_size, cell_size);
lab_env = loadPredefinedMovingEnvironment("Lab", env_size, cell_size);
normal_env = loadPredefinedMovingEnvironment("MovingReplanner", env_size, cell_size);


t = 1;

dil_dataset = dil_env.queryEnv(t);
no_stat_dataset = no_stat_env.queryEnv(t);
lab_dataset = lab_env.queryEnv(t);
normal_dataset = normal_env.queryEnv(t);
test_predicted_sdf = object_predictor.predict_object_locations(t);

map_inds = dil_dataset.map == 1;
inds = permute(dil_dataset.map, [2,1,3]) == 1;

new_sdf_patch = min(lab_dataset.field, no_stat_dataset.field);
new_map_patch = max(lab_dataset.map, no_stat_dataset.map);

fused_map = lab_dataset.map;
fused_map(map_inds) = new_map_patch(map_inds);

fused_field = lab_dataset.field;
fused_field(inds) = new_sdf_patch(inds);

diff = normal_dataset.field - fused_field;
diff_map =  fused_map - normal_dataset.map;
% any(any(any(diff_map))) Important that this is false!

predicted_map_inds = permute(test_predicted_sdf, [2,1,3]) <=0;
predicted_map = zeros(size(fused_map));
predicted_map(predicted_map_inds) = 1;


[X, Y, Z] = getEnvironmentMesh(normal_dataset);

% %Plot the fused map
% figure(2); hold on; 
% set(gcf,'Position',[1350 500 1200 1400]);
% title('Normal 3D Environment')
% grid on;
% view(3);
% gpmp2.set3DPlotRange(normal_dataset);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3DEnvironment(normal_dataset, X, Y, Z)
% 
% figure(3); hold on; 
% set(gcf,'Position',[1350 500 1200 1400]);
% title('Fused 3D Environment')
% grid on;
% view(3);
% gpmp2.set3DPlotRange(normal_dataset);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3DEnvironment(fused_map, X, Y, Z)
% 
% figure(4); hold on; 
% set(gcf,'Position',[1350 500 1200 1400]);
% title('Predicted 3D Environment')
% grid on;
% view(3);
% gpmp2.set3DPlotRange(normal_dataset);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3DEnvironment(predicted_map, X, Y, Z)
% 

height = 7;

figure(5); hold on;
subplot(1,3,1); hold on;
title('Fused Field');
h1 = plotSignedDistanceField2D(fused_field(:,:,height), origin(1), origin(2), cell_size);
subplot(1,3,2); hold on;
title('Actual');
h2 = plotSignedDistanceField2D(normal_dataset.field(:,:,height), origin(1), origin(2), cell_size);
subplot(1,3,3); hold on;
title('Diff');
h3 = plotSignedDistanceField2D(diff(:,:,height), origin(1), origin(2), cell_size);



diff = test_predicted_sdf - fused_field;

figure(6); hold on;
subplot(1,3,1); hold on;
title('Predicted');
h1 = plotSignedDistanceField2D(test_predicted_sdf(:,:,height), origin(1), origin(2), cell_size);
subplot(1,3,2); hold on;
title('Fused');
h2 = plotSignedDistanceField2D(fused_field(:,:,height), origin(1), origin(2), cell_size);
subplot(1,3,3); hold on;
title('Diff');
h3 = plotSignedDistanceField2D(diff(:,:,height), origin(1), origin(2), cell_size);




for i = 1:300
    if ~all(all(all(predicted_map(:,:,i) == normal_dataset.map(:,:,i))))
        disp(i);
    end
end







