classdef displayHSRTrajectory
    %TRAJECTORYPUBLISHER Summary of this class goes here
    %   Detailed explanation goes here
  
    properties
        publisher
        eef_path_publisher
        base_path_publisher
        delta_t
        joint_names
        node
        
        arm
        fk_model
    end
  

    methods
        function obj = displayHSRTrajectory(node)
            %TRAJECTORYPUBLISHER Construct an instance of this class
            %   Detailed explanation goes here

            obj.node = node;
            
            obj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint",...
                              "wrist_flex_joint", "wrist_roll_joint"];
                          
            obj.publisher = ros.Publisher(node,'/move_group/display_planned_path');
            obj.eef_path_publisher = ros.Publisher(node,'/path', 'nav_msgs/Path');
            obj.base_path_publisher = ros.Publisher(node,'/base_path', 'nav_msgs/Path');
            
            obj.arm = gpmp2.generateMobileArm('HSR', gtsam.Point3(0,0,0));
            obj.fk_model = obj.arm.fk_model();
            
        end
        
        function publish(obj, arr)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            num_points = size(arr, 1);
            path_msg = rosmessage('nav_msgs/Path');
            path_msg.Header.FrameId = 'world';
            
            base_path_msg = rosmessage('nav_msgs/Path');
            base_path_msg.Header.FrameId = 'world';
            
            end_eff_poses = [];
            base_poses = [];

            ro = rosmessage('moveit_msgs/DisplayTrajectory');
            po = rosmessage('moveit_msgs/RobotTrajectory');
            
            base_traj = rosmessage('trajectory_msgs/MultiDOFJointTrajectory');
            
            po.JointTrajectory.JointNames = obj.joint_names;
            base_traj.JointNames = "world_joint";
            
            traj_points = [];
            base_points = [];
            for i = 1: num_points
                eo_posestamped = rosmessage('geometry_msgs/PoseStamped');
                base_posestamped = rosmessage('geometry_msgs/PoseStamped');
                
                quat = eul2quat([0,0, arr(i, 6)], 'XYZ'); % gives [w x y z]

                jo = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                bo = rosmessage('trajectory_msgs/MultiDOFJointTrajectoryPoint');
                go = rosmessage('geometry_msgs/Transform');
                
                go.Translation.X = arr(i, 4);
                go.Translation.Y = arr(i, 5);
                go.Translation.Z = 0;

                go.Rotation.X = quat(2);
                go.Rotation.Y = quat(3);
                go.Rotation.Z = quat(4);              
                go.Rotation.W = quat(1);           
                                
                jo.TimeFromStart = rosduration(arr(i, 3));               
                bo.TimeFromStart = rosduration(arr(i, 3));    
                
                bo.Transforms = go;
                
                jo.Positions = [arr(i, 7:11)];
%                 jo.Velocities = [arr(i, 10:end), [0,0]];
                traj_points = [traj_points, jo];
                base_points = [base_points, bo];
                
                robot_pose = gpmp2.Pose2Vector(gtsam.Pose2(arr(i, 4), arr(i, 5), arr(i, 6)), arr(i, 7:11)');
                end_eff_pose = obj.fk_model.forwardKinematicsPosition(robot_pose);
                
                spheres = obj.arm.sphereCentersMat(robot_pose);
                eef_pose = spheres(:, end-4);
                
                end_eff_pose = obj.fk_model.forwardKinematicsPose(robot_pose);
                eef_quat = eul2quat([end_eff_pose(1,end), end_eff_pose(2,end), end_eff_pose(3,end)], 'ZYX'); % gives [w x y z]
          
                eo_posestamped.Header.FrameId = 'world';
                eo_posestamped.Pose.Position.X = eef_pose(1);
                eo_posestamped.Pose.Position.Y = eef_pose(2);
                eo_posestamped.Pose.Position.Z = eef_pose(3);
%                 eo_posestamped.Pose.Position.X = end_eff_pose(4,end);
%                 eo_posestamped.Pose.Position.Y = end_eff_pose(5,end);
%                 eo_posestamped.Pose.Position.Z = end_eff_pose(6,end);
                eo_posestamped.Pose.Orientation.X = eef_quat(2);
                eo_posestamped.Pose.Orientation.Y = eef_quat(3);
                eo_posestamped.Pose.Orientation.Z = eef_quat(4);
                eo_posestamped.Pose.Orientation.W = eef_quat(1);
                end_eff_poses = [end_eff_poses, eo_posestamped];

                base_posestamped.Header.FrameId = 'world';
                base_posestamped.Pose.Position.X = arr(i, 4);
                base_posestamped.Pose.Position.Y = arr(i, 5);
                base_posestamped.Pose.Position.Z = 0;
                base_posestamped.Pose.Orientation.X = 0;
                base_posestamped.Pose.Orientation.Y = 0;
                base_posestamped.Pose.Orientation.Z = 0;
                base_posestamped.Pose.Orientation.W = 1;
                
                base_poses = [base_poses, base_posestamped];

            end
            base_traj.Points = base_points;
            po.JointTrajectory.Points = traj_points;
            po.MultiDofJointTrajectory = base_traj;
            
            ro.Trajectory = po;
            
            path_msg.Poses = end_eff_poses';
            base_path_msg.Poses = base_poses';
            
            obj.publisher.send(ro);
            obj.eef_path_publisher.send(path_msg);
            obj.base_path_publisher.send(base_path_msg);
            
       end
    end
end

