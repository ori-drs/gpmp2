close all
clear all
clc
format compact

import gtsam.*
import gpmp2.*

%% Environment
%% Setup
t_start_moving = 0;
v_or_t_end = true;
use_all_straight_initialisations = false;

v_or_t_end_value = [0,-0.16, 0];
starting_pos = [0.40, 0.4, 0.4];
obs_size = [0.2, 0.2, 0.2];

env = movingEnvironment3D();
env.add_object(0,...
                v_or_t_end, ...
                v_or_t_end_value, ...
                starting_pos, ...
                obs_size);

dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);

%% This works for a single obstacle

h = figure; hold on;
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);

delta_t = 1;
for i = 0:delta_t:2 
    dataset = env.queryEnv(i);
    cla;
    h1 = plot3DEnvironment(dataset, X, Y, Z);
    centroid = regionprops(dataset.map).Centroid;
    centroid_point = cellToPoint(centroid, env);
    scatter3(centroid_point(1), centroid_point(2), centroid_point(3), 'r');
    if i>0
        px_velocity = centroid-last_centroid/delta_t;
        point_velocity = centroid_point-last_centroid_point/delta_t;
        hold on
        predicted = imtranslate(dataset.map, delta_t*px_velocity,'FillValues',255);
    end
    last_centroid = centroid;
    last_centroid_point = centroid_point;
    pause(1)
end

function point = cellToPoint(cell_ind, env)
    point = [env.dataset.origin_x, ...
                        env.dataset.origin_y,...
                        env.dataset.origin_z] + ...
                    (cell_ind * env.dataset.cell_size);
end
%% To translate the image

% imshow(imtranslate(1-dataset.map, [50, 50],'FillValues',255))

