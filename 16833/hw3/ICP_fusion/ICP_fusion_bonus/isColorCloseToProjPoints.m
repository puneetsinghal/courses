function is_close = isColorCloseToProjPoints(input_colors, proj_colors, dist2_th)

    %==== TODO: Output a boolean matrix which represents if each corresponding point pairs are close enough given dist2_th ====
    
    % Write your code here...
    difference = proj_colors- input_colors;
    is_close = sum(difference.^2,3) <= dist2_th;

end
    