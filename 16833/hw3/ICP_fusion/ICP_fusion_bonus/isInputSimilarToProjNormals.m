function is_similar = isInputSimilarToProjNormals(input_normals, proj_normals, dot_th)

    %==== TODO: Output a boolean matrix which represents if each corresponding point normals are similar enough given dot_th ====    
    
    % Write your code here...    
    dotProduct = sum(proj_normals.*input_normals,3);
    proj_normals_norm = sum(proj_normals.^2, 3).^0.5;
    input_normals_norm = sum(input_normals.^2, 3).^0.5;
    
    is_similar =  (dotProduct./(proj_normals_norm.*input_normals_norm)) >= dot_th;  

end
    