import numpy as np


def _calculate_grid_bounds(start, goal, obstacles, resolution):
    """正确计算三维网格边界"""
    all_points = [start, goal]
    
    # 展开所有障碍物的角点坐标
    for (pos, size) in obstacles:
        x, y, z = pos
        w, h, d = size
        # 障碍物的8个顶点（只需计算两个对角点即可）
        all_points.append((x, y, z))
        all_points.append((x + w, y + h, z + d))
    
    # 分离各轴坐标
    x_coords = [p[0] for p in all_points]
    y_coords = [p[1] for p in all_points]
    z_coords = [p[2] for p in all_points]
    
    # 计算各轴边界（考虑分辨率对齐）
    min_x = min(x_coords) // resolution * resolution
    max_x = (max(x_coords) // resolution + 1) * resolution
    min_y = min(y_coords) // resolution * resolution  # 修复y轴
    max_y = (max(y_coords) // resolution + 1) * resolution
    min_z = min(z_coords) // resolution * resolution  # 修复z轴
    max_z = (max(z_coords) // resolution + 1) * resolution
    
    return (min_x, max_x), (min_y, max_y), (min_z, max_z)
def _initialize_grid(bounds, obstacles, drone_size, resolution):
    (min_x, max_x), (min_y, max_y), (min_z, max_z) = bounds
    # 计算网格尺寸（确保转换为整数）
    grid_x = int((max_x - min_x) / resolution) + 1
    grid_y = int((max_y - min_y) / resolution) + 1
    grid_z = int((max_z - min_z) / resolution) + 1
    grid = np.zeros((grid_x, grid_y, grid_z), dtype=bool)
    
    # 障碍物膨胀处理（补充y/z计算）
    drone_half_w, drone_half_h, drone_half_d = [d/2 for d in drone_size]
    for (pos, size) in obstacles:
        x, y, z = pos
        w, h, d = size
        # 计算膨胀后的三维边界
        x_min = x - drone_half_w
        x_max = x + w + drone_half_w
        y_min = y - drone_half_h  # 补充y轴
        y_max = y + h + drone_half_h
        z_min = z - drone_half_d  # 补充z轴
        z_max = z + d + drone_half_d
        
        # 转换为网格坐标
        gx_min = int((x_min - min_x) / resolution)
        gx_max = int((x_max - min_x) / resolution)
        gy_min = int((y_min - min_y) / resolution)  # y轴转换
        gy_max = int((y_max - min_y) / resolution)
        gz_min = int((z_min - min_z) / resolution)  # z轴转换
        gz_max = int((z_max - min_z) / resolution)
        
        # 标记障碍区域
        grid[gx_min:gx_max+1, gy_min:gy_max+1, gz_min:gz_max+1] = True
    
    return grid