% rosinit
% node = ros.Node('/trajnode');


delta_t = 0.1;
traj_result = static_case.final_result;


traj_publisher = trajectoryPublisher(node, delta_t);
traj_publisher.publish(traj_result);
% publisher = ros.Publisher(node,'/Traj', 'trajectory_msgs/JointTrajectory');
% 
% 
% num_points = traj_result.size/2;
% traj_points = [];
% tic;
% for i = 0: num_points-1
%     po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%     po.TimeFromStart = rosduration(double(i)*delta_t);
%     po.Positions = traj_result.atVector(gtsam.symbol('x', i)');
%     po.Velocities = traj_result.atVector(gtsam.symbol('v', i)');
%     
%     traj_points = [traj_points, po];
% end
% 
% msg = rosmessage('trajectory_msgs/JointTrajectory');
% msg.Points = traj_points;
% 
% publisher.send(msg);
% toc;