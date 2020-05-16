
clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

env_name = 'MovingReplanner';
env = loadPredefinedMovingEnvironment(env_name, 300, 0.01);
    
arm = generateArm('WAMArm');
arm_model = arm.fk_model();

total_time_step = 30;
delta_t = 0.1;

dataset = env.queryEnv(0);

origin = [dataset.origin_x, ...
        dataset.origin_y, ...
        dataset.origin_z];
    
cell_size = 0.01;
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


object_predictor.update(0, env.queryEnv(0).map); 
object_predictor.update(0.5, env.queryEnv(0.5).map); 
% % profile on;
% tic;
% my_predicted_sdf = object_predictor.predict_object_locations(1);
% my_time = toc;
% % profile viewer;
% tic;
% bench_predicted_sdf = object_predictor.predict_sdf(1);
% bench_time = toc;
% 100*(1- my_time/bench_time)

test_predicted_sdf = object_predictor.predict_object_locations(1);
actual_sdf = env.queryEnv(1).field;

figure(1); hold on;
subplot(2,2,1); hold on;
h1 = plotSignedDistanceField2D(test_predicted_sdf(:,:,150), origin(1), origin(2), cell_size, epsilon);
subplot(2,2,2); hold on;
h2 = plotSignedDistanceField2D(actual_sdf(:,:,150), origin(1), origin(2), cell_size, epsilon);

% 
% 
% actual_sdf1 = env.queryEnv(1).field;
% actual_sdf2 = env.queryEnv(1.5).field;
% actual_sdf3 = env.queryEnv(2).field;
% actual_sdf1(actual_sdf1>0.2) = 1;
% actual_sdf2(actual_sdf2>0.2) = 1;
% actual_sdf3(actual_sdf3>0.2) = 1;
% actual_sdf1(actual_sdf1<0) = 0;
% actual_sdf2(actual_sdf2<0) = 0;
% actual_sdf3(actual_sdf3<0) = 0;
% 
% my_predicted_sdf1 = object_predictor.predict_object_locations(1);
% my_predicted_sdf2 = object_predictor.predict_object_locations(1.5);
% my_predicted_sdf3 = object_predictor.predict_object_locations(2);
% 
% my_predicted_sdf1(my_predicted_sdf1>0.2) = 1;
% my_predicted_sdf2(my_predicted_sdf2>0.2) = 1;
% my_predicted_sdf3(my_predicted_sdf3>0.2) = 1;
% my_predicted_sdf1(my_predicted_sdf1<0) = 0;
% my_predicted_sdf2(my_predicted_sdf2<0) = 0;
% my_predicted_sdf3(my_predicted_sdf3<0) = 0;
% 
% 
% % actual_sdf(actual_sdf>0.2) = 1;
% % my_predicted_sdf(my_predicted_sdf>0.2) = 1;
% diff_sdf = actual_sdf3-my_predicted_sdf3;
% 
% 
% figure(1); hold on;
% subplot(2,3,1); hold on;
% h1 = plotSignedDistanceField2D(actual_sdf1(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(2,3,2); hold on;
% h2 = plotSignedDistanceField2D(actual_sdf2(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(2,3,3); hold on;
% h3 = plotSignedDistanceField2D(actual_sdf3(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(2,3,4); hold on;
% h4 = plotSignedDistanceField2D(my_predicted_sdf1(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(2,3,5); hold on;
% h5 = plotSignedDistanceField2D(my_predicted_sdf2(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(2,3,6); hold on;
% h6 = plotSignedDistanceField2D(my_predicted_sdf3(:,:,150), origin(1), origin(2), cell_size, epsilon);
% 
% 
% % figure(1); hold on;
% % subplot(1,3,1); hold on;
% % h1 = plotSignedDistanceField2D(my_predicted_sdf(:,:,150), origin(1), origin(2), cell_size, epsilon);
% % subplot(1,3,2); hold on;
% % h2 = plotSignedDistanceField2D(my_predicted_sdf_3(:,:,150), origin(1), origin(2), cell_size, epsilon);
% % subplot(1,3,3); hold on;
% % h3 = plotSignedDistanceField2D(my_predicted_sdf_4(:,:,150), origin(1), origin(2), cell_size, epsilon);
% 
% figure(2); hold on;
% subplot(1,3,1); hold on;
% h1 = plotSignedDistanceField2D(my_predicted_sdf3(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(1,3,2); hold on;
% h2 = plotSignedDistanceField2D(actual_sdf3(:,:,150), origin(1), origin(2), cell_size, epsilon);
% subplot(1,3,3); hold on;
% h3 = plotSignedDistanceField2D(diff_sdf(:,:,150), origin(1), origin(2), cell_size, epsilon);
