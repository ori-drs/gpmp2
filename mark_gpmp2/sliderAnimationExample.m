% zeta = .5;                           % Damping Ratio
% wn = 2;                              % Natural Frequency
% sys = tf(wn^2,[1,2*zeta*wn,wn^2]); 
% 
% 
% f = figure;
% ax = axes('Parent',f,'position',[0.13 0.39  0.77 0.54]);
% h = stepplot(ax,sys);
% setoptions(h,'XLim',[0,10],'YLim',[0,2]);
% 
% % Add slider
% b = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
%               'value',zeta, 'min',0, 'max',1);
% % bgcolor = f.Color;
% % bl1 = uicontrol('Parent',f,'Style','text','Position',[50,54,23,23],...
% %                 'String','0','BackgroundColor',bgcolor);
% % bl2 = uicontrol('Parent',f,'Style','text','Position',[500,54,23,23],...
% %                 'String','1','BackgroundColor',bgcolor);
% % bl3 = uicontrol('Parent',f,'Style','text','Position',[240,25,100,23],...
% %                 'String','Damping Ratio','BackgroundColor',bgcolor);
% %             
% 
% % Add a callback for the animation
% b.Callback = @(es,ed) updateSystem(h,tf(wn^2,[1,2*(es.Value)*wn,wn^2])); 
% 
clear all;
close all;
clc;

x = [1:10]';
y = x.^2;
figure(1);
set_up_slider(x,y,10)

pause(3);

function set_up_slider(x, y, NDataPoints)
  h = plot( x(1), y(1), 'k');
  xlim([0,10]);
  ylim([0,100]);
  uicontrol('Style', 'slider', 'Min', 1, 'Max', 10, ...
           'Value', 1, 'SliderStep',[0.1 , 0.1], 'Position', [400 20 120 20], ...
           'Callback', @react_to_slider);
%   bl3 = uicontrol('Parent',h,'Style','text','Position',[240,25,100,23],...
%         'String','Damping Ratio','BackgroundColor',bgcolor);
  function react_to_slider(source, event)   %nested !!
      val = round(get(source, 'Value'));
      disp(val);
      set(source, 'Value', val);
      set(h, 'XData', x(1 : val), 'YData', y(1 : val));
      
  end
end
  


