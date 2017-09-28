function fusion_map = removingPoints(old_fusion_map, C_stable, t, T_max)
    
    map_ccounts = old_fusion_map.ccounts;
    map_times = old_fusion_map.times;
    
    indeces = find(map_ccounts > C_stable & ~(t - map_times > T_max));
    map_points = old_fusion_map.pointcloud.Location(indeces,:);
    
    map_colors = old_fusion_map.pointcloud.Color(indeces,:);
        
    map_normals = old_fusion_map.normals(indeces,:);
        
    map_ccounts = old_fusion_map.ccounts(indeces,:);
        
    map_times = old_fusion_map.times(indeces,:);
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);

end