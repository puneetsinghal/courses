function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);
    s = 0;

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====
    
    % Write your code here...
    K = [fx s cx;...
        0 fy cy;...
        0 0 1]; % Camera Intrinsics
    
    projection_points = pctransform(fusion_map.pointcloud, invert(tform));
        projection_frame = (K*projection_points.Location')';
    projection_frame(:,1) = round(projection_frame(:,1)./projection_frame(:,3))+1;
    projection_frame(:,2) = round(projection_frame(:,2)./projection_frame(:,3))+1;
    projection_frame(:,3) = round(projection_frame(:,3)./projection_frame(:,3));
    
    indeces = find(projection_frame(:,2)<=h & projection_frame(:,2)>0 & projection_frame(:,1)<=w & projection_frame(:,1)>0);
    proj_points = zeros(h*w,3);
    coordinates = projection_points.Location(indeces,:);
    projection_frame = [sum(coordinates.^2,2), projection_frame(indeces,:), indeces];
    projection_frame = sortrows(projection_frame);
    
    [projection_indeces, IA, ~] = unique([projection_frame(:,3), projection_frame(:,2)],'rows');
    index = sub2ind([h, w], projection_indeces(:,1), projection_indeces(:,2));
    
    proj_points(index,:) = fusion_map.pointcloud.Location(projection_frame(IA,5),:);%proj_values(:,4:6);
    proj_points = reshape(proj_points,h,w,3);
    
    proj_normals = zeros(h*w,3);
    proj_normals(index,:) = fusion_map.normals(projection_frame(IA,5),:);
    proj_normals = reshape(proj_normals,h,w,3);
    
    proj_ccounts = zeros(h*w,1);
    proj_ccounts(index) = fusion_map.ccounts(projection_frame(IA,5));
    proj_ccounts = reshape(proj_ccounts,h,w,1);
    
    proj_times = zeros(h*w,1);
    proj_times(index) = fusion_map.times(projection_frame(IA,5));
    proj_times = reshape(proj_times,h,w,1);

    proj_colors = zeros(h*w,3);
    proj_colors(index,:) = fusion_map.pointcloud.Color(projection_frame(IA,5),1:3);
    proj_colors = reshape(proj_colors,h,w,3);
    
%     indeces = find(...
%         (abs(projection_points.Location(:,1))./projection_points.Location(:,3)*fx )<=cx & ...
%         (abs(projection_points.Location(:,2))./projection_points.Location(:,3)*fy )<=cy);

%     proj_values = (K*projection_points.Location(indeces,:)')';%, fusion_map.pointcloud.Location(indeces,:)];
%     proj_values(:,1) = round(proj_values(:,1)./proj_values(:,3))+1;
%     proj_values(:,2) = round(proj_values(:,2)./proj_values(:,3))+1;
%     proj_values(:,3) = round(proj_values(:,3)./proj_values(:,3));
    
%     proj_points = zeros(h*w,3);
%     [projection_indeces, IA, ~] = unique([proj_values(:,2), proj_values(:,1)],'rows');
%     index = sub2ind([h, w], projection_indeces(:,1), projection_indeces(:,2));
    
%     proj_points(index,:) = fusion_map.pointcloud.Location(indeces(IA),:);%proj_values(:,4:6);
%     proj_points = reshape(proj_points,h,w,3);
%     
%     proj_normals = zeros(h*w,3);
%     proj_normals(index,:) = fusion_map.normals(indeces(IA),:);
%     proj_normals = reshape(proj_normals,h,w,3);
%     
%     proj_ccounts = zeros(h*w,1);
%     proj_ccounts(index) = fusion_map.ccounts(indeces(IA));
%     proj_ccounts = reshape(proj_ccounts,h,w,1);
%     
%     proj_times = zeros(h*w,1);
%     proj_times(index) = fusion_map.times(indeces(IA));
%     proj_times = reshape(proj_times,h,w,1);
% 
%     proj_colors = zeros(h*w,3);
%     proj_colors(index,:) = fusion_map.pointcloud.Color(indeces(IA),1:3);
%     proj_colors = reshape(proj_colors,h,w,3);
    
    
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
    proj_flag = zeros(size(fusion_map.pointcloud.Location, 1),1);
    proj_flag(projection_frame(IA,5)) = 1;
end