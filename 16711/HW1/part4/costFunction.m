function cost = costFunction(x)

global global_link_length global_target global_obstacles
global lowerBound upperBound hlink hjnt
global position
[frames, ee_x, ee_y, ee_z, q0, q1, q2, q3] = forwardKinematics(x);

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
    cost
    
    if (cost<0.1)
        index = size(position,2);
        position(:,index+1) = [cost; x*180/pi];
    %     image{index+1} = gcf;
        [frames, ~,~,~,~,~,~,~] = forwardKinematics(x);
        figure;
        clf
        axes
        view(3)
        axis image
        hold on
        axis([-sum(global_link_length)*1.2, sum(global_link_length)*1.2,...
            -sum(global_link_length)*1.2, sum(global_link_length)*1.2,...
            -sum(global_link_length)*1.2, sum(global_link_length)*1.2]);

        plot3(0, 0, 0, '.', 'MarkerSize', 30, 'Color', [1 0 0]);
        for i =1:length(global_link_length)
            plot3([frames(1,4,i);frames(1,4,i+1)], ...
                [frames(2,4,i);frames(2,4,i+1)], ...
                [frames(3,4,i);frames(3,4,i+1)], ...
                'Color', [0 0 0.7], 'LineWidth', 5);
            plot3(frames(1,4,i+1), frames(2,4,i+1), frames(3,4,i+1),'.', 'MarkerSize', 30, 'Color', [1 0 0]);
        end
        plot3(global_target(1), global_target(2), global_target(3),'.', 'MarkerSize', 30, 'Color', [0 0 0]);
        
        [x_sphere,y,z] = sphere;

        for i = 1:size(global_obstacles,1)
            surf(x_sphere*global_obstacles(i,4)+global_obstacles(i,1), ...
            y*global_obstacles(i,4)+global_obstacles(i,2), ...
            z*global_obstacles(i,4)+global_obstacles(i,3));
        end

    end
end
