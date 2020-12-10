classdef kinovaTrajectoryPublisher
    %TRAJECTORYPUBLISHER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pub
        delta_t
        joint_names
        rGoalMsg
        rArm
    end
  

    methods
        function obj = kinovaTrajectoryPublisher(delta_t)
            %TRAJECTORYPUBLISHER Construct an instance of this class
            %   Detailed explanation goes here

            obj.delta_t = delta_t;
            
            obj.joint_names = ["j2n6s300_joint_1", ... 
                               "j2n6s300_joint_2", ...
                               "j2n6s300_joint_3",...
                              "j2n6s300_joint_4", ...
                              "j2n6s300_joint_5", ...
                              "j2n6s300_joint_6"];
            obj.pub = rospublisher('/j2n6s300_driver/trajectory_controller/command', ...
                                   'trajectory_msgs/JointTrajectory');
                               
                               
            [obj.rArm, obj.rGoalMsg] = rosactionclient('/j2n6s300/follow_joint_trajectory');
            waitForServer(obj.rArm);
            
            obj.rGoalMsg.Trajectory.JointNames = obj.joint_names;
 
            % Disable feedback
            obj.rArm.FeedbackFcn = [];
            obj.rArm.ActivationFcn = [];
            obj.rArm.ResultFcn = [];
        end
        
        function publish(obj,traj, curr_step)
            num_points = traj.size/2;
            traj_points = [];
            
            for i = 0: num_points-1 - curr_step
                po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                po.TimeFromStart = rosduration(double(i)*obj.delta_t);               
                po.Positions = [traj.atVector(gtsam.symbol('x', curr_step + i))'];
                po.Velocities = [traj.atVector(gtsam.symbol('v', curr_step + i))'];
                traj_points = [traj_points, po];
            end

            msg = rosmessage(obj.pub);
            msg.Points = traj_points;
            msg.JointNames = obj.joint_names';
            disp(msg);
            obj.pub.send(msg);
        end
       
       function kinovaTest(obj)
            
            po = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            po.TimeFromStart = rosduration(0);               
            po.Positions = [0,0,0,0,0,0]; 
            po.Velocities = zeros(1,6);
             
            p1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            p1.TimeFromStart = rosduration(2);               
            p1.Positions = [0,0.5,0,0,0,0]; 
            p1.Velocities = [0,0,0,0,0,0];

            p2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            p2.TimeFromStart = rosduration(5);               
            p2.Positions = [0,0,0,0,0,0]; 
            p2.Velocities = [0,0,0,0,0,0];
            
            obj.rGoalMsg.Trajectory.Points = [po, p1, p2];

            sendGoal(obj.rArm,obj.rGoalMsg);
       end
       

    end
end



