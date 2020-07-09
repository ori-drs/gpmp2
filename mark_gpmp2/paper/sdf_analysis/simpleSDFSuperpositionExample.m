
% @author Mark Finean 
% @date May 27, 2020

close all;
clear all;
clc;
    
import gpmp2.*
import gtsam.*

env_size = 300;
dataset_size = [env_size,env_size,env_size];
cell_size = 0.01;
origin = [-1.5,-1.5,-1.5];
epsilon = 0.1;

fullmap = zeros(dataset_size);
labmap = zeros(dataset_size);
obs1_map = zeros(dataset_size);
obs2_map = zeros(dataset_size);
obs3_map = zeros(dataset_size);

% Add lab sphere
lab_sphere_center = [0, 0, 0];
lab_sphere_radius = 0.5;
inds = getSphereInds(lab_sphere_radius, lab_sphere_center, origin, cell_size, env_size);
fullmap(inds) = 1;
labmap(inds) = 1;

% Add obs sphere 1
obs_sphere_1_center = [-0.8, 0.4, 0];
obs_sphere_1_radius = 0.2;
inds = getSphereInds(0.2, obs_sphere_1_center, origin, cell_size, env_size);
fullmap(inds) = 1;
obs1_map(inds) = 1;

% Add obs sphere 2
obs_sphere_2_center = [0.8, 0, 0];
obs_sphere_2_radius = 0.2;
inds = getSphereInds(0.2, obs_sphere_2_center, origin, cell_size, env_size);
fullmap(inds) = 1;
obs2_map(inds) = 1;

% Add obs sphere 3
obs_sphere_3_center = [-0.6, -0.6, 0];
obs_sphere_3_radius = 0.2;
inds = getSphereInds(0.2, obs_sphere_3_center, origin, cell_size, env_size);
fullmap(inds) = 1;
obs3_map(inds) = 1;

% Now calculate fields
full_field  = gpmp2.signedDistanceField3D(fullmap, cell_size); 
obs1_field  = gpmp2.signedDistanceField3D(obs1_map, cell_size); 
obs2_field  = gpmp2.signedDistanceField3D(obs2_map, cell_size); 
obs3_field  = gpmp2.signedDistanceField3D(obs3_map, cell_size); 
lab_field  = gpmp2.signedDistanceField3D(labmap, cell_size); 


%% Superposition

obs1_rect_inds = getRectObstacleMask(obs_sphere_1_center, obs_sphere_1_radius, ...
                                    epsilon, origin, cell_size, env_size);
obs2_rect_inds = getRectObstacleMask(obs_sphere_2_center, obs_sphere_2_radius, ...
                                    epsilon, origin, cell_size, env_size);
obs3_rect_inds = getRectObstacleMask(obs_sphere_3_center, obs_sphere_3_radius, ...
                                    epsilon, origin, cell_size, env_size);   
                                
fused_field = lab_field;
fused_field(obs1_rect_inds) = min(obs1_field(obs1_rect_inds), fused_field(obs1_rect_inds));
fused_field(obs2_rect_inds) = min(obs2_field(obs2_rect_inds), fused_field(obs2_rect_inds));
fused_field(obs3_rect_inds) = min(obs3_field(obs3_rect_inds), fused_field(obs3_rect_inds));
% fused_field = fliplr(fused_field); 

% Needed regions
needed_map = full_field < epsilon;

%% Plots

% Plot the full map
figure(1); hold on;
imshow(~fullmap(:,:,150)')

% Plot the real sdf
figure(2); hold on;
gpmp2.plotSignedDistanceField2D(permute(full_field(:,:,150), [2 1 3]), -1.5, -1.5, cell_size);
colormap(jet)

% Plot the real sdf
figure(3); hold on;
gpmp2.plotSignedDistanceField2D(permute(fused_field(:,:,150), [2 1 3]), -1.5, -1.5, cell_size);
colormap(jet)

% Plot the necessary sdf regions
figure(4); hold on;
imshow(~needed_map(:,:,150)')





















function inds = getRectObstacleMask(centre, r, epsilon, origin, cell_size, env_size)
    min_coord = centre - r - epsilon;
    max_coord = centre + r + epsilon;

    x_vals = min_coord(1):cell_size:max_coord(1);
    y_vals = min_coord(2):cell_size:max_coord(2);
    z_vals = min_coord(3):cell_size:max_coord(3);

    [X, Y, Z] = ndgrid(x_vals, y_vals, z_vals);
    inds = pointsToInds(X, Y, Z, origin, cell_size, env_size);
end

function inds = pointsToInds(x, y, z, origin, cell_size, env_size)
    
    X = reshape(x, 1, []);
    Y = reshape(y, 1, []);
    Z = reshape(z, 1, []);
    points = vertcat(X,Y,Z)';
    
    obj_cell_coords = points;
    obj_cell_coords(:,1) = round((obj_cell_coords(:,1) - origin(1)) / cell_size) + 1;
    obj_cell_coords(:,2) = round((-obj_cell_coords(:,2) + origin(2)) / cell_size) + env_size;
    obj_cell_coords(:,3) = round((obj_cell_coords(:,3) - origin(3)) / cell_size) + 1;

    inds = sub2ind([env_size,env_size,env_size],...
                    obj_cell_coords(:,1)', ...
                    obj_cell_coords(:,2)', ...
                    obj_cell_coords(:,3)');
end

function inds = getSphereInds(sphere_radius, centre, origin, cell_size, env_size)

    coords = -sphere_radius:cell_size:sphere_radius;
    [X, Y, Z] = ndgrid(coords + centre(1), coords + centre(2), coords + centre(3));
    in_sphere = (X - centre(1)).^2 + (Y - centre(2)).^2 + (Z - centre(3)).^2 <= sphere_radius.^2;
    
%     in_sphere = reshape(in_sphere, 1, []);

    X = X(in_sphere);
    Y = Y(in_sphere);
    Z = Z (in_sphere);
    
%     X = reshape(X, 1, []);
%     Y = reshape(Y, 1, []);
%     Z = reshape(Z, 1, []);
%     points = vertcat(X,Y,Z)';

%     points = points(in_sphere,:);
    inds = pointsToInds(X, Y, Z, origin, cell_size, env_size);
    
%     obj_cell_coords = points;
%     obj_cell_coords(:,1) = round((obj_cell_coords(:,1) - origin(1)) / cell_size) + 1;
%     obj_cell_coords(:,2) = round((-obj_cell_coords(:,2) + origin(2)) / cell_size) + env_size;
%     obj_cell_coords(:,3) = round((obj_cell_coords(:,3) - origin(3)) / cell_size) + 1;
% 
%     inds = sub2ind([env_size,env_size,env_size],...
%                     obj_cell_coords(:,1)', ...
%                     obj_cell_coords(:,2)', ...
%                     obj_cell_coords(:,3)');
end