function [nonLinearConstraint,ceq] = sphereCollision(x)

global global_link_length global_obstacles
[frames, ~, ~, ~, ~, ~, ~, ~] = forwardKinematics(x);

for i = 1:length(global_link_length)
    for j = 1:size(global_obstacles,1)
        nonLinearConstraint(i*j,1) = checkIntersection([frames(1:3,4,i), frames(1:3,4,i+1)],...
            global_obstacles(j,:));
    end
end

% nonLinearConstraint = [c1; c2; c3];
ceq = [];

end

function value = checkIntersection(lineSegment, sphere)

a = (lineSegment(1,2) - lineSegment(1,1))^2 + ...
    (lineSegment(2,2) - lineSegment(2,1))^2 + ...
    (lineSegment(3,2) - lineSegment(3,1))^2;

b = 2*((lineSegment(1,2) - lineSegment(1,1))*(lineSegment(1,1) - sphere(1)) + ...
    (lineSegment(2,2) - lineSegment(2,1))*(lineSegment(2,1) - sphere(2)) +...
    (lineSegment(3,2) - lineSegment(3,1))*(lineSegment(3,1) - sphere(3)));

c = sphere(1)^2 + sphere(2)^2 + sphere(3)^2 + ...
    lineSegment(1,1)^2 + lineSegment(2,1)^2 + lineSegment(3,1)^2 - ...
    2*(sphere(1)*lineSegment(1,1) + sphere(2)*lineSegment(2,1) + sphere(3)*lineSegment(3,1)) - ...
    sphere(4)^2 ;

determinant = b * b - 4 * a * c;
if (determinant)<0
    value = determinant + 0.01 ;
else
    t1 = (-b + sqrt(determinant))/2/a;
    t2 = (-b - sqrt(determinant))/2/a;
    t = max(abs(t1), abs(t2));
    value = (1.1 - t);
end

end