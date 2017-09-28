function [r, p, y, fval] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
%% Function that uses analytic gradients to do optimization for inverse kinematics in a snake robot

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


options = optimoptions('fmincon',...
    'Algorithm','interior-point', 'SpecifyObjectiveGradient',true,...
    'MaxIterations',300, ...
    'OptimalityTolerance', 0.01, ...
    'MaxFunctionEvaluations', 6000);

A = [];
b = [];
Aeq = [];
beq = [];
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

nonlcon = @(x)sphereCollision(x);
fun = @(x)rosentwo(x);
[x, f_val] = fmincon(fun,...
    initialJoints, A, b, Aeq, beq, ...
    lowerBound, upperBound, @(x)sphereCollision(x), options);
r = x(1:length(x)/3);
p = x(length(x)/3+1:length(x)*2/3);
y = x(2/3*length(x)+1:end);
if (nargout ==4)
    fval = f_val;
end
end

function [f,g] = rosentwo(x)
    global global_target
    f = costFunction(x);
    if nargout > 1 % gradient required
        epsilon = 0.01;
        for i=1:length(x)
                [~, ee_x, ee_y, ee_z, q0, q1, q2, q3] = forwardKinematics(x);
                del_x = x;
                del_x(i) = del_x(i)+epsilon;
                [~, del_ee_x, del_ee_y, del_ee_z, del_q0, del_q1, del_q2, del_q3] = forwardKinematics(del_x);
                delx_delg(i) = (del_ee_x - ee_x)/epsilon;
                dely_delg(i) = (del_ee_y - ee_y)/epsilon;
                delz_delg(i) = (del_ee_z - ee_z)/epsilon;
                delq0_delg(i) = (del_q0 - q0)/epsilon;
                delq1_delg(i) = (del_q1 - q1)/epsilon;
                delq2_delg(i) = (del_q2 - q2)/epsilon;
                delq3_delg(i) = (del_q3 - q3)/epsilon;
        end
        g = -(2*((global_target(1) - ee_x)*delx_delg + ...
            (global_target(2) - ee_y)*dely_delg + ...
            (global_target(3) - ee_z)*delz_delg) + ...
            2*((global_target(4) - q0)*delq0_delg + (global_target(5) - q1)*delq1_delg + ...
            (global_target(6) - q2)*delq2_delg + (global_target(7) - q3)*delq3_delg));
    end
end