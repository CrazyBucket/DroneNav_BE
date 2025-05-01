from typing import Dict, List, Tuple
from core.a_star_3d import a_star_3d
from core.grid_utils import _calculate_grid_bounds, _initialize_grid
from core.obstacle_processor import parse_obstacles


def plan_path(
        current_pos: Tuple[float, float, float],
        target_pos: Tuple[float, float, float],
        scene_config: Dict,
        drone_size: Tuple[float, float, float],
        grid_resolution: float = 0.5
) -> List[Tuple[float, float, float]]:
    # 解析场景配置中的障碍物
    obstacles = parse_obstacles(scene_config)
    
    # 计算三维网格边界
    grid_bounds = _calculate_grid_bounds(
        start=current_pos,
        goal=target_pos,
        obstacles=obstacles,
        resolution=grid_resolution
    )

    # 坐标转换函数
    def to_grid(pos: Tuple[float, float, float]) -> Tuple[int, int, int]:
        return (
            int((pos[0] - grid_bounds[0][0]) / grid_resolution),
            int((pos[1] - grid_bounds[1][0]) / grid_resolution),
            int((pos[2] - grid_bounds[2][0]) / grid_resolution)
        )

    # 转换起点终点到网格坐标
    start_grid = to_grid(current_pos)
    goal_grid = to_grid(target_pos)

    # 初始化碰撞网格
    grid = _initialize_grid(
        bounds=grid_bounds,
        obstacles=obstacles,
        drone_size=drone_size,
        resolution=grid_resolution
    )

    # A* 路径搜索
    path_grid = a_star_3d(
        start=start_grid,
        goal=goal_grid,
        grid=grid,
        resolution=grid_resolution
    )

    # 转换回实际坐标
    path_world = [
        (
            grid_bounds[0][0] + x * grid_resolution,
            grid_bounds[1][0] + y * grid_resolution,
            grid_bounds[2][0] + z * grid_resolution
        )
        for (x, y, z) in path_grid
    ]

    return path_world