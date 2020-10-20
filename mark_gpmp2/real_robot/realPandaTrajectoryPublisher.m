classdef realPandaTrajectoryPublisher
    %TRAJECTORYPUBLISHER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        publisher
        delta_t
        joint_names
        rGoalMsg
        rArm
    end
  

    methods
        function obj = realPandaTrajectoryPublisher(delta_t)
            %TRAJECTORYPUBLISHER Construct an instance of this class
            %   Detailed explanation goes here

            obj.delta_t = delta_t;
            
            obj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3",...
                              "panda_joint4", "panda_joint5", "panda_joint6", ...
                              "panda_joint7"];
            [obj.rArm, obj.rGoalMsg] = rosactionclient('/position_joint_trajectory_controller/follow_joint_trajectory');
            waitForServer(obj.rArm);
            
            obj.rGoalMsg.Trajectory.JointNames = obj.joint_names;
 
            % Disable feedback
            obj.rArm.FeedbackFcn = [];
            obj.rArm.ActivationFcn = [];
            obj.rArm.ResultFcn = [];
        end
        
%         function publish(obj,traj, curr_step, curr_conf, curr_vel)
        function publish(obj,traj, curr_step, lag_steps)
            num_points = traj.size/2;
            traj_points = [];
%             
%             po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%             po.TimeFromStart = rosduration(0);               
%             po.Positions = curr_conf';
%             po.Velocities = curr_vel';
%             traj_points = [traj_points, po];
%                 
            for i = lag_steps: num_points-1 - curr_step
                po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                po.TimeFromStart = rosduration(double(i)*obj.delta_t);               
                po.Positions = [traj.atVector(gtsam.symbol('x', curr_step + i))'];
                po.Velocities = [traj.atVector(gtsam.symbol('v', curr_step + i))'];
                traj_points = [traj_points, po];
            end

            obj.rGoalMsg.Trajectory.Points = traj_points;

            sendGoal(obj.rArm,obj.rGoalMsg);
        end
       
       function goToConfig(obj,config)
            
            po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            po.TimeFromStart = rosduration(3);               
            po.Positions = config;
            po.Velocities = zeros(1,7);

            obj.rGoalMsg.Trajectory.Points = po;

            sendGoal(obj.rArm,obj.rGoalMsg);
       end

       
       function cancelGoals(obj)
            cancelAllGoals(obj.rArm);
       end
       
    end
end

