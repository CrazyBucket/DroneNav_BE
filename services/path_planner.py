from typing import List, Tuple
from core.a_star_3d import a_star_3d
from core.grid_utils import _calculate_grid_bounds, _initialize_grid


def plan_path(
        current_position: Tuple[float, float, float],
        target_position: Tuple[float, float, float],
        obstacles: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = None,
        drone_size: Tuple[float, float, float] = (0.30, 0.40, 0.10),
        grid_resolution: float = 0.5
) -> List[Tuple[float, float, float]]:
    # 计算网格物理边界（使用实际坐标）
    grid_bounds = _calculate_grid_bounds(
        current_position,
        target_position,
        obstacles or [],
        grid_resolution
    )

    # 坐标转换函数（基于网格边界）
    def to_grid(pos):
        x = (pos[0] - grid_bounds[0][0]) / grid_resolution
        y = (pos[1] - grid_bounds[1][0]) / grid_resolution
        z = (pos[2] - grid_bounds[2][0]) / grid_resolution
        return int(x), int(y), int(z)

    # 转换起点终点
    start = to_grid(current_position)
    goal = to_grid(target_position)

    # 初始化网格
    grid = _initialize_grid(grid_bounds, obstacles or [], drone_size, grid_resolution)

    # 运行A*算法
    path_grid = a_star_3d(start, goal, grid, grid_resolution)

    # 转换回实际坐标
    path = [
        (
            grid_bounds[0][0] + x * grid_resolution,
            grid_bounds[1][0] + y * grid_resolution,
            grid_bounds[2][0] + z * grid_resolution
        )
        for (x, y, z) in path_grid
    ]
    return path