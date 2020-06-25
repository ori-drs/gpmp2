classdef trajectoryPublisher
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
        function obj = trajectoryPublisher(delta_t)
            %TRAJECTORYPUBLISHER Construct an instance of this class
            %   Detailed explanation goes here

            obj.delta_t = delta_t;
            
            obj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3",...
                              "panda_joint4", "panda_joint5", "panda_joint6", ...
                              "panda_joint7"];
            [obj.rArm, obj.rGoalMsg] = rosactionclient('/panda_arm_controller/follow_joint_trajectory');
            waitForServer(obj.rArm);
            
            obj.rGoalMsg.Trajectory.JointNames = obj.joint_names;
 
            % Disable feedback
            obj.rArm.FeedbackFcn = [];
            obj.rArm.ActivationFcn = [];
            obj.rArm.ResultFcn = [];
        end
        
        function publish(obj,traj, curr_step)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            num_points = traj.size/2;
            traj_points = [];
            
            for i = 0: num_points-1 - curr_step
                po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                po.TimeFromStart = rosduration(double(i)*obj.delta_t);               
                po.Positions = [traj.atVector(gtsam.symbol('x', curr_step + i))'];
                po.Velocities = [traj.atVector(gtsam.symbol('v', curr_step + i))'];
                traj_points = [traj_points, po];
            end

            obj.rGoalMsg.Trajectory.Points = traj_points;

            sendGoal(obj.rArm,obj.rGoalMsg);
       end
    end
end

