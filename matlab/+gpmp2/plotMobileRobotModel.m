function h = plotMobileRobotModel(marm, p, vehsize, color, width)
%PLOTPLANNARMOBILEARM Summary of this function goes here
%   Detailed explanation goes here

import gtsam.*
import gpmp2.*

fk_model = marm.fk_model();

% pose = p.pose();
% % vehicle corners
% corner1 = pose.transformFrom(Point2(vehsize(1)/2, vehsize(2)/2));
% corner2 = pose.transformFrom(Point2(-vehsize(1)/2, vehsize(2)/2));
% corner3 = pose.transformFrom(Point2(-vehsize(1)/2, -vehsize(2)/2));
% corner4 = pose.transformFrom(Point2(vehsize(1)/2, -vehsize(2)/2));
% 
% vehicle base black lines
% h(1) = plot([corner1.x() corner2.x() corner3.x() corner4.x() corner1.x()], ...
%     [corner1.y() corner2.y() corner3.y() corner4.y() corner1.y()], 'k-');
% 
% arm
% position = fk_model.forwardKinematicsPosition(p);
% position = position(1:2, :);
% 
% style = strcat(color, '-');
% h(2) = plot(position(1,:), position(2,:), style, 'LineWidth', width);
% 
% h(3) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 5);
% 


color_rgb = [0.4 0.4 0.4];

% points
body_points = marm.sphereCentersMat(p);

% show
colormap(color_rgb);
[X_ball, Y_ball, Z_ball] = sphere(16);

for i=1:marm.nr_body_spheres()
    h(i) = surf(X_ball * marm.sphere_radius(i-1) + body_points(1, i), ...
                Y_ball * marm.sphere_radius(i-1) + body_points(2, i), ...
                Z_ball * marm.sphere_radius(i-1) + body_points(3, i));
end

end