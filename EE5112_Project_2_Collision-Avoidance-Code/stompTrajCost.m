% Given a trajectory, calculate its cost
function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta,  R, voxel_world, poeData)
% Compute the local trajectory cost at each discretization theta point, as 
% well as the overall trajecotry cost (the Qtheta)

% Costi = stompCompute_Cost(robot, theta, Env);
% Compute the cost of all discretized points on one trajectory
[~, nDiscretize] = size(theta);
% Obstacle cost
qo_cost = zeros(1, nDiscretize);
% Constraint costs
qc_cost = zeros(1, nDiscretize);

% Get the coordinates of joints in World frame 
[X, T] = updateJointsWorldPosition(robot_struct, theta(:, 1), poeData);
% Construct the spheres around the robot manipulator for collision
% avoidance
[sphere_centers,radi] = stompRobotSphere(X);
% Initial velocity at the sphere centers around the manipulator is 0
vel = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);
qc_cost(1) = orientationConstraintCost(T{end});

for i = 2 : nDiscretize
    sphere_centers_prev = sphere_centers;
    % Calculate the kinematics of the manipulator, given the
    % configuration theta values at different time (i=2:nDiscretize)
    [X, T] = updateJointsWorldPosition(robot_struct, theta(:, i), poeData);
    [sphere_centers, radi] = stompRobotSphere(X);
    % xb: 3D workspace position of sphere b at the current time
    % Approximate the speed (xb_dot) using the finite difference of the current and
    % the previous position
    vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
    qo_cost(i) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);
    
    %% Orientation constraint to keep end-effector y-axis upright
    qc_cost(i) = orientationConstraintCost(T{end});
end

%% Local trajectory cost: you need to specify the relative weights between different costs
Stheta = 1000*qo_cost + qc_cost;

% sum over time and add the smoothness cost
theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end


function penalty = orientationConstraintCost(T_ee)
desiredAxis = [0;0;1];
eeYAxis = T_ee(1:3,2);
penalty = 100 * norm(eeYAxis - desiredAxis, 1);
end
    
