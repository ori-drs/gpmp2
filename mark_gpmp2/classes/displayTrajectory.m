classdef displayTrajectory
    %TRAJECTORYPUBLISHER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        publisher
        eef_path_publisher
        delta_t
        joint_names
        node
        arm
        fk_model
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
            obj.eef_path_publisher = ros.Publisher(node,'/path', 'nav_msgs/Path');
             
            obj.arm = gpmp2.generateArm('Panda', gtsam.Pose3(gtsam.Rot3(eye(3)), gtsam.Point3([0,0,0]')));
            obj.fk_model = obj.arm.fk_model();
        end
        
        function publish(obj, traj, curr_step)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            num_points = traj.size/2;
           
            path_msg = rosmessage('nav_msgs/Path');
            path_msg.Header.FrameId = 'world';
            
            ro = rosmessage('moveit_msgs/DisplayTrajectory');
            po = rosmessage('moveit_msgs/RobotTrajectory');
            po.JointTrajectory.JointNames = obj.joint_names;
            
            end_eff_poses = [];

            traj_points = [];
            for i = 0: num_points-1 - curr_step
                
                eo_posestamped = rosmessage('geometry_msgs/PoseStamped');

                curr_conf = traj.atVector(gtsam.symbol('x', curr_step + i))';
                end_eff_pose = obj.fk_model.forwardKinematicsPosition(curr_conf');
                spheres = obj.arm.sphereCentersMat(curr_conf');
                eef_pose = spheres(:, end-4);
                eef_quat = eul2quat([end_eff_pose(1,end), end_eff_pose(2,end), end_eff_pose(3,end)], 'ZYX'); % gives [w x y z]
          
                eo_posestamped.Header.FrameId = 'world';
                eo_posestamped.Pose.Position.X = eef_pose(1);
                eo_posestamped.Pose.Position.Y = eef_pose(2);
                eo_posestamped.Pose.Position.Z = eef_pose(3);

                eo_posestamped.Pose.Orientation.X = eef_quat(2);
                eo_posestamped.Pose.Orientation.Y = eef_quat(3);
                eo_posestamped.Pose.Orientation.Z = eef_quat(4);
                eo_posestamped.Pose.Orientation.W = eef_quat(1);
                end_eff_poses = [end_eff_poses, eo_posestamped];
                
                jo = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                jo.TimeFromStart = rosduration(double(i)*obj.delta_t);               
                jo.Positions = traj.atVector(gtsam.symbol('x', curr_step + i))';
                jo.Velocities = traj.atVector(gtsam.symbol('v', curr_step + i))';
                traj_points = [traj_points, jo];
            end
            
            po.JointTrajectory.Points = traj_points;
            path_msg.Poses = end_eff_poses';

            ro.Trajectory = po;
            
            obj.eef_path_publisher.send(path_msg);
            obj.publisher.send(ro);
       end
    end
end

