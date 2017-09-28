function is_close = isInputCloseToProjPoints(input_points, proj_points, dist2_th)

    %==== TODO: Output a boolean matrix which represents if each corresponding point pairs are close enough given dist2_th ====
    
    % Write your code here...
    difference = proj_points - input_points;
    is_close = sum(difference.^2,3) <= dist2_th;

end
    