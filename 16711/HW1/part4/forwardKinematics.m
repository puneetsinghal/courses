function [frames, ee_x, ee_y, ee_z, q0, q1, q2, q3] = forwardKinematics(x)

global global_link_length
dof = length(global_link_length);

frames = zeros(4,4,dof);
jointAngles = [x(1:length(x)/3),x(length(x)/3+1:2*length(x)/3),x(2*length(x)/3+1:end)];

for i = 1:dof+1
    if (i>1)
        frames(1:3,:,i) = cat(2, ...
            rotx(jointAngles(i-1,1))*roty(jointAngles(i-1,2))*rotz(jointAngles(i-1,3)),...
            [0; 0; 0]);
        frames(4,:,i) = [0 0 0 1];
        frames(:,:,i) = frames(:,:,i)*[1 0 0 global_link_length(i-1); 0 1 0 0;...
            0 0 1 0;0 0 0 1];
        frames(:,:,i) = frames(:,:,i-1)*frames(:,:,i);
    end   
    
    if (i==1)
        frames(:,:,i) = eye(4);
    end
    
end

% disp(frames);


ee_x = frames(1,4,end);
ee_y = frames(2,4,end);
ee_z = frames(3,4,end);

Q = rotm2quat(frames(1:3,1:3,end));
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);

end

function rotationMatrix = rotx(theta)
    rotationMatrix = [1 0 0;
        0 cos(theta) -sin(theta);
        0 sin(theta) cos(theta)];
end

function rotationMatrix = roty(theta)
    rotationMatrix = [cos(theta) 0 sin(theta);
        0 1 0;
        -sin(theta) 0 cos(theta)];
end

function rotationMatrix = rotz(theta)
    rotationMatrix = [cos(theta) -sin(theta) 0;
        sin(theta) cos(theta) 0;
        0 0 1];
end

