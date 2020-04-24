close all
clear all
clc
format compact

import gtsam.*
import gpmp2.*

plot_figs = false;

%% Environment setup
t_start_moving = 0;
v_or_t_end = true;
use_all_straight_initialisations = false;

v_or_t_end_value1 = [0,-0.05, 0];
v_or_t_end_value2 = [0, 0.05, 0];

starting_pos1 = [0.40, 0.4, 0.4];
starting_pos2 = [0.40, -0.2, 0.4];
obs_size = [0.2, 0.2, 0.2];

env = movingEnvironment3D();

% Add block 1
env.add_object(0,...
                v_or_t_end, ...
                v_or_t_end_value1, ...
                starting_pos1, ...
                obs_size);

% Add block 2
env.add_object(0,...
    v_or_t_end, ...
    v_or_t_end_value2, ...
    starting_pos2, ...
    obs_size);

            
dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);
dataset_size = size(dataset.map);
%% Visualise the environment

if plot_figs
    figure(1); hold on;
    sgtitle('3D Environment - Aerial view')
    subplot(1,2,1);
    title('t'); grid on, view(3)
    gpmp2.set3DPlotRange(dataset);
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3DEnvironment(env.queryEnv(0), X, Y, Z)
    view(0,90);
    
    subplot_figsplot(1,2,2)
    title('t+1'); grid on, view(3)
    gpmp2.set3DPlotRange(dataset);
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3DEnvironment(env.queryEnv(1), X, Y, Z)
    view(0,90);
end

% Find all the centroids
conn_comps = bwconncomp(dataset.map);
regions = regionprops(conn_comps,'Centroid');
num_obs = conn_comps.NumObjects;
obs_centroids = zeros(num_obs, 3);
for i = 1:num_obs
    obs_centroids(i, :) = regions(i).Centroid;
end

%% Plot

h = figure(2); hold on;
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);

t_predict = 15;
% Make one time step to predict
delta_t = 1;
for t = 0:delta_t:1 
    dataset = env.queryEnv(t);
    cla; hold on;
%     h1 = plot3DEnvironment(dataset, X, Y, Z);
    
    conn_comps = bwconncomp(dataset.map);
    regions = regionprops(conn_comps,'Centroid');
    num_obs = conn_comps.NumObjects;

    obs_centroids = zeros(num_obs, 3);
    obs_centroids_points = zeros(num_obs, 3);
    obj_coords = cell(num_obs);
    
    predicted_env = zeros(300,300,300);
    % For each object get centroids
    for j = 1:num_obs
        % Get centroid
        tic;
        centroid = regions(j).Centroid;
        obs_centroids(j, :) = centroid;
        
        centroid_point = cellToPoint(centroid, env);
        obs_centroids_points(j, :) = centroid_point;
        
        % Plot Centroid
        scatter3(centroid_point(1), centroid_point(2), centroid_point(3), 'r');
        
        if t>0
            % Matching with closest point in last frame
            [~, min_ind] = min(sum((centroid - last_centroids).^2,2));
            
            px_velocity = (obs_centroids(j,:)-last_centroids(min_ind,:))./delta_t;
            point_velocity = (obs_centroids_points(j,:)-last_centroid_points(min_ind,:))/delta_t;
    
            px_list = conn_comps.PixelIdxList{j};
            % Get all pixel coords
%             [row,col,z] = ind2sub(dataset_size,conn_comps.PixelIdxList{j});
%             obj_coords{j} = horzcat(row,col,z);
            
            predicted_occupancy_inds = getPredictedOccupancyInds(t_predict-t,...
                                                                px_velocity,...
                                                                dataset_size,...
                                                                px_list);
                                                            
            predicted_env(predicted_occupancy_inds) = 1;

%         hold on
%         predicted = imtranslate(dataset.map, delta_t*px_velocity,'FillValues',255);
        end
        toc
    end
    last_centroids = obs_centroids;
    last_centroid_points = obs_centroids_points;

        
    pause(1)
end

%% Plot prediction and actual
% if plot_figs

figure(1); hold on;
sgtitle('3D Environment - Aerial view')
subplot(1,2,1);
title("Actual at time: " + num2str(t_predict)); grid on, view(3)
gpmp2.set3DPlotRange(dataset);
xlabel('x'); ylabel('y'); zlabel('z');
plot3DEnvironment(env.queryEnv(t_predict), X, Y, Z);
view(0,90);
% % view(3);

subplot(1,2,2);
title("Predicted at time: " + num2str(t_predict)); grid on, view(3)
gpmp2.set3DPlotRange(dataset);
xlabel('x'); ylabel('y'); zlabel('z');
plot3DEnvironment(predicted_env, X, Y, Z)
view(0,90);
% view(3);

% subplot(1,3,3);
% title("Actual at time: " + num2str(t)); grid on, view(3)
% gpmp2.set3DPlotRange(dataset);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3DEnvironment(env.queryEnv(t), X, Y, Z)
% view(0,90);
% view(3);
% end


%%

% test = randi(5, 4, 4, 3);
% test = zeros(2, 4, 3);
% test(1,1,1) = 1;
% px_velocity = [1,0,0];
% px_list = [1 2];
% px_inds = getPredictedOccupancyInds(1, px_velocity, size(test), px_list);

function px_inds = getPredictedOccupancyInds(time, px_velocity, dataset_size, px_list)

    px_velocity_mod = [px_velocity(2),px_velocity(1),px_velocity(3)];
    
    [row,col,z] = ind2sub(dataset_size,px_list);
    obj_coords = horzcat(row,col,z);
    predicted_coords = obj_coords + (px_velocity_mod*time);
    
    % Remove the coords out of the workspace
    keep_mask = all(predicted_coords >= [1,1,1]  & predicted_coords <= dataset_size, 2); % any row that is out of bounds
    predicted_coords = predicted_coords(keep_mask, :);
    
    px_inds = sub2ind(dataset_size,predicted_coords(:,1)', predicted_coords(:,2)', predicted_coords(:,3)');
end
            
function point = cellToPoint(cell_ind, env)
    point = [env.dataset.origin_x, ...
                        env.dataset.origin_y,...
                        env.dataset.origin_z] + ...
                    (cell_ind * env.dataset.cell_size);
end
