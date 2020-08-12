function h = plotEndEffectorTrajectory(arm, traj, color)

    time_steps = traj.size/2 -1;
    positions = zeros(time_steps, 3);
    
    for i = 0:time_steps
        conf = traj.atVector(gtsam.symbol('x', i));
        position = arm.forwardKinematicsPosition(conf);
        positions(i+1,:) = position(:,end)';
    end
    
    h(1) = plot3(positions(:,1), positions(:,2), positions(:,3), '-', ...
    'Color',color, 'MarkerSize', 5, 'LineWidth', 2);
        
end
