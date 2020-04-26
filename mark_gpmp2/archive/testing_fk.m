% 
% @author Mark Finean 
% @date April 13, 2020

close all;
clear all;
clc;
% profile on

import gtsam.*
import gpmp2.*


%% Problem setup
up_conf = [0,0,0,0,0,0,0]';
forward_conf = [0,1.57,0,0,0,0,0]';

start_conf = up_conf;
end_conf = forward_conf;

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

% initial values by batch
position = arm_model.forwardKinematicsPosition(up_conf);

figure();
hold on;
axis([0 1 -0.5 0.5 0 1]);
grid on, view(3)
plotArm(arm.fk_model(), up_conf, 'r', 2)

env = movingEnvironment3D();
% Add block 1
env.add_object(0,...
                true, ...
                [0,-0.05, 0], ...
                [0,1,0], ...
                [0.02,0.02, 0.02]);
            
origin = [env.dataset.origin_x, ...
        env.dataset.origin_y, ...
        env.dataset.origin_z];

dataset = env.queryEnv(0);
multiple_pos = [[0,0,0]; [0,1,0]; [1,0,0]; [1,1,0]];
% multiple_pos = [0,1,0];

coord = positionToCoord(multiple_pos, origin, env.dataset.rows, env.dataset.cell_size);
query_inds = sub2ind(size(dataset.map),coord(:,1)', coord(:,2)', coord(:,3)');
 
% function coord = positionToCoord(pos, origin, rows, cell_size)
%     coord = round([pos(1) - origin(1), ...
%                     -pos(2) + origin(2), ...
%                     pos(3) - origin(3)]/cell_size) ...
%                     + [1, rows,1];
% end
inds = find(dataset.map ==1);
[row,col,z] = ind2sub(size(dataset.map),inds);
obj_coords = horzcat(row,col,z);

function swapped_coord = positionToCoord(positions, origin, rows, cell_size)
    positions(:,2) = -positions(:,2);
    origin(:,2) = - origin(:,2);

    coord = round((positions - origin)/cell_size) + [1, rows ,1];
    swapped_coord = zeros(size(coord));
    swapped_coord(:,1) = coord(:,2);
    swapped_coord(:,2) = coord(:,1);
    swapped_coord(:,3) = coord(:,3);
end
