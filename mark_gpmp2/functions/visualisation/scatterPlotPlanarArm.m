function h = scatterPlotPlanarArm(position, color, edge_alpha, face_alpha, handle)
%PLOTPLANARARM Plot Arm class in 2D
%
%   Usage: PLOTPLANARARM(arm, conf, color, width)
%   @arm    Arm object
%   @conf   arm configuration vector
%   @color  color string, use plot convention, e.g. 'r' is red
%   @width  line width

% position = arm.forwardKinematicsPosition(conf);
position = position(1:2, :);
position = [[0;0], position];

joint_X = position(1,:);
joint_Y = position(2,:);

interp_X = [];
interp_Y = [];
for i = 1:size(joint_X,2)-1
    XI = linspace(joint_X(i),joint_X(i+1),10);
    YI = interp1(joint_X(1,i:i+1),joint_Y(1,i:i+1) ,XI);
    
    interp_X = [interp_X, XI];
    interp_Y = [interp_Y, YI];

end

% h(1) = plot(joint_X, joint_Y, style, 'LineWidth', width);
% 
% h(2) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 20);

scatter(handle, interp_X, interp_Y, ...
        'MarkerEdgeColor', color, ...
        'MarkerFaceColor', color, ...
        'MarkerFaceAlpha', face_alpha,...
        'MarkerEdgeAlpha', edge_alpha)
end

