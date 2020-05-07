clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

conf = [0,0,0,0,0,0,0]';
replanner_start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
replanner_end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';
conf = replanner_start_conf;

% Set up figure
f = figure(1); hold on;
ax = gca;

% Load arm
arm = generateArm('WAMArm');
% arm_model = arm.fk_model();

% Initial plot
axis([-1 1.5 -1.2 1.5 -1 2]);
grid on; view(3);
title("WAM Testing");
xlabel('x'); ylabel('y'); zlabel('z');
plotRobotModel(arm, conf);



% num2str(k*.1/2)
for k=1:length(conf)
    eval(['uicontrol(''style'',''slider'',''units'',''norm'',''pos'',[.1 ',num2str(k*.1/2),' .1 .05 ],''callback'',@slm,''min'',-50,''max'',50,''value'',',num2str(conf(k)),',''sliderstep'',[.001 .05])'])
    eval(['uicontrol(''style'',''text'',''fontname'',''calibri'',''fontsize'',14,''fontangle'',''italic'',''units'',''norm'',''backgroundcolor'',''w'',''pos'',[.2 ',num2str(k*.1/2),' .05 .05 ],''string'',',...
        '[''d',num2str(k),'''])'])
end

function slm(varargin)

    arm = gpmp2.generateArm('WAMArm');

    ui_control_list =flipud(findobj('type','uicontrol','style','slider'));
    cla;
    hold on

    for k=1:length(ui_control_list)
        conf(k)=get(ui_control_list(k),'value');
    end
        
    gpmp2.plotRobotModel(arm, conf');
    
    axis([-1 1.5 -1.2 1.5 -1 2]);
    grid on; 
%     view(3);
    title("WAM Testing");
    xlabel('x'); ylabel('y'); zlabel('z');
    disp(conf);
   

end