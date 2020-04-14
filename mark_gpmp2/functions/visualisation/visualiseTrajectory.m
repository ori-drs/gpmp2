function visualiseTrajectory(arm_fk, conf_list)
%VISUALISETRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
    for i=1:size(conf_list,2)
        conf = conf_list(i);               
        position = arm_fk.forwardKinematicsPosition(conf);
        position = position(1:2, :);
%         position = [[0;0], position];

        
        position = [[0;0], position];
        color = 'r';
        width = 2;
        style = strcat(color, '-');
        
        %  Plots the lines
        h(1) = plot(position(1,:), position(2,:), style, 'LineWidth', width);
        
        % Plots the joints
        h(2) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 20);


        % Plots the joints
%         c = ['r','b'];
%         for i = 1:size(position,2)
% %             h(i) = scatter(position(1,i), position(2,i),...
% %                 'MarkerFaceColor',c(i),...
% %                 'MarkerEdgeColor',c(i),...
% %                 'MarkerFaceAlpha',.2,...
% %                 'MarkerEdgeAlpha',.2);
%         end
%         h(2).Color(4) = 0.1;
  
        hold on;
    end
end

