classdef displaySavedTrajectory
    %TRAJECTORYPUBLISHER Summary of this class goes here
    %   Detailed explanation goes here
  
    properties
        publisher
        path_publisher
        delta_t
        joint_names
        node
        
        arm
        fk_model
    end
  

    methods
        function obj = displaySavedTrajectory(node, delta_t)
            %TRAJECTORYPUBLISHER Construct an instance of this class
            %   Detailed explanation goes here

            obj.delta_t = delta_t;
            obj.node = node;
            
            obj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3",...
                              "panda_joint4", "panda_joint5", "panda_joint6", ...
                              "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"];
                          
            obj.publisher = ros.Publisher(node,'/move_group/display_planned_path');
            obj.path_publisher = ros.Publisher(node,'/path', 'nav_msgs/Path');
            
            obj.arm = gpmp2.generateArm('Panda', gtsam.Pose3(gtsam.Rot3(eye(3)), gtsam.Point3([0,0,0.4]')));
            obj.fk_model = obj.arm.fk_model();
            
        end
        
        function publish(obj, arr)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            num_points = size(arr, 1);
            path_msg = rosmessage('nav_msgs/Path');
            path_msg.Header.FrameId = 'world';
            link_8_poses = [];

            ro = rosmessage('moveit_msgs/DisplayTrajectory');
            po = rosmessage('moveit_msgs/RobotTrajectory');
            po.JointTrajectory.JointNames = obj.joint_names;
            
            traj_points = [];
            for i = 1: num_points
                eo_posestamped = rosmessage('geometry_msgs/PoseStamped');

                jo = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                jo.TimeFromStart = rosduration(arr(i, 2) * obj.delta_t);               
                jo.Positions = [arr(i, 3:9), [0.04,0.04]];
                jo.Velocities = [arr(i, 10:end), [0,0]];
                traj_points = [traj_points, jo];
                
                pose = obj.fk_model.forwardKinematicsPosition(arr(i, 3:9)');
                
                eo_posestamped.Header.FrameId = 'world';
                eo_posestamped.Pose.Position.X = pose(1,end);
                eo_posestamped.Pose.Position.Y = pose(2,end);
                eo_posestamped.Pose.Position.Z = pose(3,end);
                eo_posestamped.Pose.Orientation.X = 0;
                eo_posestamped.Pose.Orientation.Y = 0;
                eo_posestamped.Pose.Orientation.Z = 0;
                eo_posestamped.Pose.Orientation.W = 1;
                link_8_poses = [link_8_poses, eo_posestamped];
            end
            
            po.JointTrajectory.Points = traj_points;
            
            ro.Trajectory = po;
            
            path_msg.Poses = link_8_poses';
            
            obj.publisher.send(ro);
            obj.path_publisher.send(path_msg);
       end
    end
end

