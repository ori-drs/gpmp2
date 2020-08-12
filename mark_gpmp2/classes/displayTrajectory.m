classdef displayTrajectory
    %TRAJECTORYPUBLISHER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        publisher
        delta_t
        joint_names
        node
    end
  

    methods
        function obj = displayTrajectory(node, delta_t)
            %TRAJECTORYPUBLISHER Construct an instance of this class
            %   Detailed explanation goes here

            obj.delta_t = delta_t;
            obj.node = node;
            
            obj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3",...
                              "panda_joint4", "panda_joint5", "panda_joint6", ...
                              "panda_joint7"];
                          
            obj.publisher = ros.Publisher(node,'/move_group/display_planned_path');
            
        end
        
        function publish(obj, traj, curr_step)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            num_points = traj.size/2;
           
            ro = rosmessage('moveit_msgs/DisplayTrajectory');
            po = rosmessage('moveit_msgs/RobotTrajectory');
            po.JointTrajectory.JointNames = obj.joint_names;
            
            traj_points = [];
            for i = 0: num_points-1 - curr_step
                jo = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                jo.TimeFromStart = rosduration(double(i)*obj.delta_t);               
                jo.Positions = traj.atVector(gtsam.symbol('x', curr_step + i))';
                jo.Velocities = traj.atVector(gtsam.symbol('v', curr_step + i))';
                traj_points = [traj_points, jo];
            end
            
            po.JointTrajectory.Points = traj_points;
            
            ro.Trajectory = po;
            
            obj.publisher.send(ro);
       end
    end
end

