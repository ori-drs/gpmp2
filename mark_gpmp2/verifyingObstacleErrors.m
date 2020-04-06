% planar arm obstacle avoidance example with moving obstacle, build factor graph in matlab
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 18, 2020

close all;
clear;
clc;

import gtsam.*
import gpmp2.*



%% Defined all Environments
v_or_t_end = true;
% obs_size = [0.60, 0.80];
obs_size = [0.2, 1];

top_left_env = movingEnvironment(0,v_or_t_end,[0, 0],[-1.0, 1.0], obs_size);
top_left_dataset = top_left_env.queryEnv(0);

top_centre_env = movingEnvironment(0,v_or_t_end,[0, 0],[0, 1.0], obs_size);
top_centre_dataset = top_centre_env.queryEnv(0);

top_right_env = movingEnvironment(0,v_or_t_end,[0, 0],[1.0, 1.0], obs_size);
top_right_dataset = top_right_env.queryEnv(0);

centre_left_env = movingEnvironment(0,v_or_t_end,[0, 0],[-1.0, 0], obs_size);
centre_left_dataset = centre_left_env.queryEnv(0);

centre_centre_env = movingEnvironment(0,v_or_t_end,[0, 0],[0, 0], obs_size);
centre_centre_dataset = centre_centre_env.queryEnv(0);
% centre_centre_env = movingEnvironment(0,v_or_t_end,[0, 0],[0, -0.], obs_size);
% centre_centre_dataset = centre_centre_env.queryEnv(0);

centre_right_env = movingEnvironment(0,v_or_t_end,[0, 0],[1.0, 0], obs_size);
centre_right_dataset = centre_right_env.queryEnv(0);

bottom_left_env = movingEnvironment(0,v_or_t_end,[0, 0],[-1.0, -1.0], obs_size);
bottom_left_dataset = bottom_left_env.queryEnv(0);

bottom_centre_env = movingEnvironment(0,v_or_t_end,[0, 0],[0, -1.0], obs_size);
bottom_centre_dataset = bottom_centre_env.queryEnv(0);

bottom_right_env = movingEnvironment(0,v_or_t_end,[0, 0],[1.0, -1.0], obs_size);
bottom_right_dataset = bottom_right_env.queryEnv(0);

datasets = [top_left_dataset, top_centre_dataset, top_right_dataset, ...
            centre_left_dataset, centre_centre_dataset, centre_right_dataset, ...
            bottom_left_dataset, bottom_centre_dataset, bottom_right_dataset];
           
%% plot sdfs
% figure(1)
% sgtitle("Signed distance fields")
% subplot(3,3,1);
% plotSignedDistanceField2D(flip(top_left_dataset.field), top_left_dataset.origin_x, top_left_dataset.origin_y, top_left_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% subplot(3,3,2);
% plotSignedDistanceField2D(flip(top_centre_dataset.field), top_centre_dataset.origin_x, top_centre_dataset.origin_y, top_centre_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% subplot(3,3,3);
% plotSignedDistanceField2D(flip(top_right_dataset.field), top_right_dataset.origin_x, top_right_dataset.origin_y, top_right_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% 
% subplot(3,3,4);
% plotSignedDistanceField2D(flip(centre_left_dataset.field), centre_left_dataset.origin_x, centre_left_dataset.origin_y, centre_left_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% 
% subplot(3,3,5);
% plotSignedDistanceField2D(flip(centre_centre_dataset.field), centre_centre_dataset.origin_x, centre_centre_dataset.origin_y, centre_centre_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% 
% subplot(3,3,6);
% plotSignedDistanceField2D(flip(centre_right_dataset.field), centre_right_dataset.origin_x, centre_right_dataset.origin_y, centre_right_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% 
% subplot(3,3,7);
% plotSignedDistanceField2D(flip(bottom_left_dataset.field), bottom_left_dataset.origin_x, bottom_left_dataset.origin_y, bottom_left_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% 
% subplot(3,3,8);
% plotSignedDistanceField2D(flip(bottom_centre_dataset.field), bottom_centre_dataset.origin_x, bottom_centre_dataset.origin_y, bottom_centre_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% 
% subplot(3,3,9);
% plotSignedDistanceField2D(flip(bottom_right_dataset.field), bottom_right_dataset.origin_x, bottom_right_dataset.origin_y, bottom_right_dataset.cell_size);
% % title('Left');
% xlim([-1,1]);
% ylim([-1,1]);
% 
% hold off;


%% Add the arm
cost_sigma = 0.9;
epsilon_dist = 0.01;

% arm model
arm = gpmp2.generateArm('SimpleTwoLinksArm');
arm_model = arm.fk_model();

key_pos = gtsam.symbol('x', 0);
key_vel = gtsam.symbol('v', 0);

test_env = movingEnvironment(0,v_or_t_end,[0, 0],[0, -0.5], obs_size);
test_dataset = test_env.queryEnv(0);

right_conf = [0, 0]';
top_conf = [pi/2, 0]';
left_conf = [pi, 0]';
bottom_conf = [3*pi/2, 0]';

arm_positions = [right_conf, top_conf, left_conf, bottom_conf];

arm_values = [];
for i = 1:size(arm_positions,2)
    init_values = gtsam.Values;
    init_values.insert(key_pos, arm_positions(:,i));
    init_values.insert(key_vel, [0, 0]');
    arm_values = [arm_values, init_values];
end

% %for each sdf scenario
% % for j = 1:size(datasets, 2)
% % for j = 4:6
% for j = 5
% % for j = 1:3
%     % for each arm config
%     
%     % plot start / end configuration
%     figure(j+1);
%     hold on;
% 
%     for i = 1:size(arm_positions,2)
% %     for i = 4
%         subplot(2,2,i);
%         hold on;
%         customPlotEvidenceMap2D(datasets(j).map, datasets(j).origin_x, datasets(j).origin_y, datasets(j).cell_size);
%         % Calculate the error
%         obs_factor = gpmp2.ObstaclePlanarSDFFactorArm(key_pos, arm, datasets(j).sdf, cost_sigma, epsilon_dist);
%         error = obs_factor.error(arm_values(i));
%         plotPlanarArm(arm.fk_model(), arm_positions(:, i), 'b', 2);
%         title("Error: " + num2str(error));
% %         arm.fk_model().forwardKinematicsPosition(arm_positions(i))
%     end
%     pause(1);
% end

        
% plot start / end configuration
figure(10);
hold on;

for i = 1:size(arm_positions,2)
%     for i = 4
    subplot(2,2,i);
    hold on;
    customPlotEvidenceMap2D(test_dataset.map, test_dataset.origin_x, ...
        test_dataset.origin_y, test_dataset.cell_size);
    
    % Calculate the error
    obs_factor = gpmp2.ObstaclePlanarSDFFactorArm(key_pos, arm, ...
                        test_dataset.sdf, cost_sigma, epsilon_dist);
    error = obs_factor.error(arm_values(i));
    obs_factor.evaluateError([0;0])

    plotPlanarArm(arm.fk_model(), arm_positions(:, i), 'b', 2);
    title("Error: " + num2str(error) + "eval Err: " + ...
        num2str(sum(obs_factor.evaluateError(arm_positions(:, i)))));
end

obs_factor.evaluateError([0;0])
obs_factor.evaluateError([pi/2;0])
obs_factor.evaluateError([pi;0])
obs_factor.evaluateError([3*pi/2;0])






