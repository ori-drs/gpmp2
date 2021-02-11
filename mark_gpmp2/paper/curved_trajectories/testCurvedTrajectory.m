close all;
clear all;
clc;

e = 0;
speed = 1;
total_time_sec = 3;
delta_t = 0.1;

trajCalculator = curvedTrajectory(total_time_sec, speed, e);
b = trajCalculator.b;

figure(1);
subplot(3,1,1); xlabel('t'); ylabel('x'); xlim([0, total_time_sec]); ylim([0, b + 0.4]); hold on;
subplot(3,1,2); xlabel('t'); ylabel('y'); xlim([0, total_time_sec]); ylim([-b, b]); hold on;
subplot(3,1,3); xlabel('y'); ylabel('x'); ylim([0.4, b + 0.4]); xlim([-b, b]); hold on;


num_pos = length(0:delta_t:total_time_sec);
positions = zeros(num_pos,2);

i = 1;
for t = 0:delta_t:total_time_sec
   pos = trajCalculator.getPosition(t);
   positions(i,1) = pos(1);
   positions(i,2) = pos(2);
   subplot(3,1,1);
   scatter(t,pos(1));
   subplot(3,1,2);
   scatter(t,pos(2));
   subplot(3,1,3);
   scatter(pos(2),pos(1));
%    pause(0.2);
   i= i + 1;
end


figure(2); hold on; xlim([0, 3]); ylim([-3, 3]);
pbaspect([1 2 1]);

for speed = 0.5:0.1:2.0
    trajCalculator = curvedTrajectory(total_time_sec, speed, e);

    num_pos = length(0:delta_t:total_time_sec);
    positions = zeros(num_pos,2);

    i = 1;
    for t = 0:delta_t:total_time_sec
       pos = trajCalculator.getPosition(t);
       positions(i,1) = pos(1); positions(i,2) = pos(2);
       i= i + 1;
    end

    plot(positions(:,1), positions(:,2));
end