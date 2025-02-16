from typing import List, Tuple
import numpy as np
from core.a_star_3d import a_star_3d
from core.grid_utils import _calculate_grid_bounds, _initialize_grid


def plan_path(
    current_position: Tuple[float, float, float],
    target_position: Tuple[float, float, float],
    obstacles: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
    drone_size: Tuple[float, float, float] = (1.0, 1.0, 1.0),
    grid_resolution: float = 1.0,
) -> List[Tuple[float, float, float]]:
    """
    三维A*路径规划函数
    
    参数:
        current_position: 当前坐标(x, y, z)
        target_position: 目标坐标(x, y, z)
        obstacles: 障碍物列表，每个元素为(位置(x,y,z), 尺寸(w,h,d))
        drone_size: 无人机尺寸(w, h, d)
        grid_resolution: 网格分辨率（米）
    
    返回:
        路径坐标列表，从起点到终点
    """
    # 转换为网格坐标
    def to_grid(pos):
        return (int(pos[0]/grid_resolution), int(pos[1]/grid_resolution), int(pos[2]/grid_resolution))
    
    start = to_grid(current_position)
    goal = to_grid(target_position)
    
    # 计算膨胀后的障碍物网格
    grid_bounds = _calculate_grid_bounds(start, goal, obstacles, grid_resolution)
    grid = _initialize_grid(grid_bounds, obstacles, drone_size, grid_resolution)
    
    # 运行A*算法
    path_grid = a_star_3d(start, goal, grid, grid_resolution)
    
    # 转换回实际坐标
    path = [(x*grid_resolution, y*grid_resolution, z*grid_resolution) for (x,y,z) in path_grid]
    return path