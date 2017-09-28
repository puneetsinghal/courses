function is_close = isInputCloseToProjPoints(input_points, proj_points, dist2_th)

    %==== TODO: Output a boolean matrix which represents if each corresponding point pairs are close enough given dist2_th ====
    
    % Write your code here...
    proj_points_reshaped = reshape(proj_points,[],3);
    input_points_reshaped = reshape(input_points,[],3);
    difference = proj_points_reshaped - input_points_reshaped;
    is_close = sum(difference.^2,2) <= dist2_th;
    is_close = reshape(is_close,size(input_points,1),size(input_points,2));

end
    