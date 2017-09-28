function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====
    
    % Write your code here...
    indeces = find(proj_flag==0);
    map_points = fusion_map.pointcloud.Location(indeces,:);
    map_points = [map_points;reshape(updated_map.points, [], 3)];
    
    map_colors = fusion_map.pointcloud.Color(indeces,:);
    map_colors = [map_colors;reshape(updated_map.colors, [], 3)];
        
    map_normals = fusion_map.normals(indeces,:);
    map_normals = [map_normals;reshape(updated_map.normals, [], 3)];
        
    map_ccounts = fusion_map.ccounts(indeces,:);
    map_ccounts = [map_ccounts;reshape(updated_map.ccounts, [], 1)];
        
    map_times = fusion_map.times(indeces,:);
    map_times = [map_times;reshape(updated_map.times, [], 1)];
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   