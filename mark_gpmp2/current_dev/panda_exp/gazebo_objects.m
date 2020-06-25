
clear all;
close all;
clc;

import gtsam.*
node = ros.Node('/matlab_node');


cell_size = 0.01;
env_size = 300;
origin = [-1,-1,-1];
epsilon = 0.2;

arm = myGenerateArm('Panda', Pose3(Rot3(eye(3)), Point3([0, 0, 0.5]')));
arm_model = arm.fk_model();



sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
pub = ros.Publisher(node,'/start_simulation','std_msgs/String');
strMsg = rosmessage('std_msgs/String');

env = liveEnvironment(node, env_size, cell_size, origin);
tracker = objectTrackerPredictor([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon, cell_size, env.dataset.origin_point3);
                            
pause(2);

% Start simulation                            
pub.send(strMsg); tic;                            
                         
env.updateMap();
current_t = toc;
tracker.update(current_t, env.dataset.map);

current_t = toc;
tracker.update(current_t, env.dataset.map);


% start_conf =  [1.57,           0.185,   0,       -1.70,   0,        3.14,   0]';
current_joint_msg = sub.LatestMessage;
start_conf = current_joint_msg.Position(3:end);


[X, Y, Z] = getEnvironmentMesh(env.dataset);
% plot problem setting
figure(1); hold on;
title('3D Environment')
grid on, view(3)
axis([-1 2 -1 2 -1 2]);
xlabel('x');
ylabel('y');
zlabel('z');
plot3DEnvironment(env.dataset, X, Y, Z)
hold on;
gpmp2.plotRobotModel(arm, start_conf)
pause(1);