function [r, p, y, fval] = part3( target, link_length, ...
    min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles)
%% Function that uses optimization to do inverse kinematics for a snake robot

%%Outputs 
  % [r, p, y] = roll, pitch, yaw vectors of the N joint angles
  %            (N link coordinate frames)
%%Inputs:
    % target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
    %    effector
    % link_length : Nx1 vectors of the lengths of the links
    % min_xxx, max_xxx are the vectors of the 
    %    limits on the roll, pitch, yaw of each link.
    % limits for a joint could be something like [-pi, pi]
    % obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
    %    obstacle. M obstacles.

% Your code goes here.
% [x, y, z, Q] = forwardKinematics(link_length,jointAngles);



global global_obstacles global_link_length global_target
global_obstacles = obstacles;
global_target = target;
global_link_length = link_length;

global lowerBound upperBound
lowerBound = [min_roll; min_pitch; min_yaw];
upperBound = [max_roll; max_pitch; max_yaw];


for i = 1:size(global_obstacles,1)
    dist = (global_target(1) - global_obstacles(i,1))^2 + ...
        (global_target(2) - global_obstacles(i,2))^2 + ...
        (global_target(3) - global_obstacles(i,3))^2;
    if (dist<=global_obstacles(i,4)^2+0.1)
        error('target lies inside obstacle');
        break;
    end
end

initialJoints = ones(length(global_link_length)*3,1);

global hlink hsphere hjnt htarget
figure;
clf
axes
view(3)
axis image
hold on
axis([-sum(global_link_length)*1.2, sum(global_link_length)*1.2,...
    -sum(global_link_length)*1.2, sum(global_link_length)*1.2,...
    -sum(global_link_length)*1.2, sum(global_link_length)*1.2]);

hjnt(1) = plot3(0, 0, 0, '.', 'MarkerSize', 30, 'Color', [1 0 0]);
for i =1:length(global_link_length)
    hlink(i) = plot3([0; global_link_length(i)], [0; 0], [0; 0], 'Color', [0 0 0.7], 'LineWidth', 5);
    hjnt(i+1) = plot3(global_link_length(i), 0, 0,'.', 'MarkerSize', 30, 'Color', [1 0 0]);
end
htarget = plot3(global_target(1), global_target(2), global_target(3),'.', 'MarkerSize', 30, 'Color', [0 0 0]);
[x_sphere,y,z] = sphere;

for i = 1:size(global_obstacles,1)
    hsphere(i) = surf(x_sphere*global_obstacles(i,4)+global_obstacles(i,1), ...
    y*global_obstacles(i,4)+global_obstacles(i,2), ...
    z*global_obstacles(i,4)+global_obstacles(i,3));
end
opts.LBounds = lowerBound;
opts.UBounds = upperBound;

[xmin, f_val] = cmaes('costFunction', initialJoints, 0.4, opts);

r = xmin(1:length(xmin)/3);
p = xmin(length(xmin)/3+1:length(xmin)*2/3);
y = xmin(2/3*length(xmin)+1:end);
if (nargout ==4)
    fval = f_val;
end


end