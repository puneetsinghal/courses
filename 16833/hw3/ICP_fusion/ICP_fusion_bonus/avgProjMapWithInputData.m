function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha_old, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    updated_points = proj_map.points;
    updated_colors = proj_map.colors;
    updated_normals = proj_map.normals;
    updated_ccounts = proj_map.ccounts;
    updated_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====   
    
    % Write your code here...
    new_indeces = find(is_use == 1);
    [m, n] = find(is_use == 1);
    
    proj_ccounts = repmat(reshape(proj_map.ccounts,[],1),1,3);
    alpha = repmat(reshape(alpha_old,[],1), 1, 3);
    input_points = reshape(input_points,[],3);
    input_normals = reshape(input_normals,[],3);
    input_colors = reshape(input_colors,[],3);
    
    
    updated_points = reshape(updated_points,[],3);
    updated_points(new_indeces, :) = ...
        (proj_ccounts(new_indeces,:).*updated_points(new_indeces, :) +...
        alpha(new_indeces, :).*input_points(new_indeces, :))./...
        (proj_ccounts(new_indeces,:) + alpha(new_indeces, :));
    updated_points = reshape(updated_points,h,w,3);

    updated_normals = reshape(updated_normals,[],3);
    updated_normals(new_indeces, :) =...
        (proj_ccounts(new_indeces, :).*updated_normals(new_indeces, :) +...
        alpha(new_indeces, :).*input_normals(new_indeces, :))./...
        (proj_ccounts(new_indeces, :) + alpha(new_indeces, :));
    updated_normals = reshape(updated_normals,h,w,3);

    updated_ccounts = reshape(updated_ccounts,[],1);
    updated_ccounts(new_indeces) = updated_ccounts(new_indeces) + alpha(new_indeces, 1);
    updated_ccounts = reshape(updated_ccounts,h,w,1);
    
    updated_times = reshape(updated_times,[],1);
    updated_times(new_indeces) = t;
    updated_times = reshape(updated_times,h,w,1);
    
    updated_colors = reshape(updated_colors,[],3);
    updated_colors(new_indeces,:) = input_colors(new_indeces,:);
    updated_colors = reshape(updated_colors,h,w,3);

    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end