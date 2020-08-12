import gpmp2.*

arm = generateArm('Panda');

figure(1)
hold on;
conf = [0,          -0.785,   0,       -2.356,   0,        1.57,   0]';
gpmp2.plotRobotModel(arm, conf);
view(3)
xlim([-2,2]);ylim([-2,2]);zlim([-2,2]);
rotate3d on
ax = gca;
enableDefaultInteractivity(ax)
