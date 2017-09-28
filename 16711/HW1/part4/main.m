% link_length = [1 1 1 1 1 1]';
% numOfObstacles = 4;
% obstacles = 0.5*sum(link_length)*rand(numOfObstacles,3);
% obstacles = [obstacles, (rand(numOfObstacles,1)+ones(numOfObstacles,1))/2];
% min_roll = -pi*ones(size(link_length));
% max_roll = pi*ones(size(link_length));
% min_pitch = -pi*ones(size(link_length));
% max_pitch = pi*ones(size(link_length));
% min_yaw = -pi*ones(size(link_length));
% max_yaw = pi*ones(size(link_length));
% 
% target = [3, 3, 2, cos(pi/4), 1/sqrt(3)*sin(pi/4), 1/sqrt(3)*sin(pi/4), 1/sqrt(3)*sin(pi/4)];

load('input.mat');

global position
position = [];
tic 
[r, p, y] = part2(target, link_length, ...
    min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);
toc
[~, ee_x, ee_y, ee_z, q0, q1, q2, q3] = forwardKinematics([r;p;y]);
position2 = [ee_x, ee_y, ee_z, q0, q1, q2, q3]
error_part2 = ((target-position2)*...
    (target-position2)')/norm(target)