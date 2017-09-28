function cost = costFunction(x)

global global_link_length global_target
global lowerBound upperBound hlink hjnt

PROMPT = 'Enter for next iteration';
input(PROMPT);
[frames, ~, ~, ~, q0, q1, q2, q3] = forwardKinematics(x);

cost = (((global_target(1) - frames(1,4,end))^2 + ...
    (global_target(2) - frames(2,4,end))^2 + (global_target(3) - frames(3,4,end))^2) + ...
    ((global_target(4) - q0)^2 + (global_target(5) - q1)^2 + ...
    (global_target(6) - q2)^2 + (global_target(7) - q3)^2)) + ...
    0.001/min((x-lowerBound)'*(x-lowerBound),(x-upperBound)'*(x-upperBound));

for i = 1: length(global_link_length)
    set(hlink(i),...
        'XData', [frames(1,4,i);frames(1,4,i+1)],...
        'YData', [frames(2,4,i);frames(2,4,i+1)],...
        'ZData', [frames(3,4,i);frames(3,4,i+1)]);
    set(hjnt(i+1), 'XData', frames(1,4,i+1),'YData', frames(2,4,i+1),'ZData', frames(3,4,i+1));
end
drawnow

[nonLinearConstraint,~] = sphereCollision(x);

flag = 0;

for i = 1:length(nonLinearConstraint)
    if (nonLinearConstraint(i)>0)
        flag = 1;
        break;
    end
end

if (flag==1)
    cost = NaN;
end

end
