
import gtsam.*
import gpmp2.*

% node = ros.Node('/matlab_node');

sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
pause(2);

current_joint_msg = sub.LatestMessage;

start_conf = current_joint_msg.Position(3:end);
% end_conf = [1.57,           0.185,   0,       -1.70,   0,        3.14,   0]';
end_conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';

env = loadPredefinedMovingEnvironment('Empty', 150, 0.02, [-1,-1,-1]);
dataset = env.queryEnv(0);

arm = generateArm('Panda', Pose3(Rot3(eye(3)), Point3([0,-0.1,-0.5]')));
arm_model = arm.fk_model();

total_time_sec = 3.0;
delta_t = 0.1;
interp_multiplier = 20;
cost_sigma = 0.05;
epsilon_dist = 0.2;    
limit_v = false;
limit_x = false;

% Planner settings
total_time_step = round(total_time_sec/delta_t);


problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v);


% get datasets
datasets = [];

for i = 0:total_time_step
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   

init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

static_case = staticUpdateCase(datasets, init_values, problem_setup);
% full_case = fullKnowledgeUpdateCase(datasets, init_values, problem_setup);
% update_case = updateCase(datasets, init_values, problem_setup);


traj_publisher = trajectoryPublisher(delta_t);
traj_publisher.publish(static_case.final_result);
