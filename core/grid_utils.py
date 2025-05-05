# core/grid_utils.py
import numpy as np

from config.settings import OBSTACLE_BUFFER


def _calculate_grid_bounds(start, goal, obstacles, resolution, buffer=3, drone_size=(1,1,1)):
    """计算网格边界（不包含安全缓冲，安全缓冲由_initialize_grid处理）"""
    all_points = [start, goal]
    
    # 更合理的安全缓冲系数
    buffer_multiplier = 1.2  # 降低安全缓冲系数到1.2（解决过度膨胀问题）
    
    # 预计算安全缓冲
    safe_margin = [
        (drone_size[0]/2 + resolution) * buffer * buffer_multiplier,
        (drone_size[1]/2 + resolution) * buffer * buffer_multiplier,
        (drone_size[2]/2 + resolution) * buffer * buffer_multiplier
    ]

    for pos, size in obstacles:
        x, y, z = pos
        w, h, d = size
        
        # 扩展障碍物边界以包含安全缓冲
        expanded_min = (
            x - safe_margin[0],
            y - safe_margin[1],
            z - safe_margin[2]
        )
        expanded_max = (
            x + w + safe_margin[0],
            y + h + safe_margin[1],
            z + d + safe_margin[2]
        )
        all_points.extend([expanded_min, expanded_max])

    # 分离各轴坐标
    x_coords = [p[0] for p in all_points]
    y_coords = [p[1] for p in all_points]
    z_coords = [p[2] for p in all_points]

    # 增加额外空间
    extra_space = resolution * 8  # 增加更多额外空间，确保有足够空间绕行

    # 计算各轴边界（考虑分辨率对齐）
    min_x = np.floor((min(x_coords) - extra_space) / resolution) * resolution
    max_x = np.ceil((max(x_coords) + extra_space) / resolution) * resolution
    min_y = np.floor((min(y_coords) - extra_space) / resolution) * resolution
    max_y = np.ceil((max(y_coords) + extra_space) / resolution) * resolution
    min_z = np.floor((min(z_coords) - extra_space) / resolution) * resolution
    max_z = np.ceil((max(z_coords) + extra_space) / resolution) * resolution

    return (min_x, max_x), (min_y, max_y), (min_z, max_z)


def _initialize_grid(bounds, obstacles, drone_size, resolution, buffer=3):
    (min_x, max_x), (min_y, max_y), (min_z, max_z) = bounds

    # 更合理的安全缓冲系数
    buffer_multiplier = 1.2  # 降低安全缓冲系数到1.2（解决过度膨胀问题）

    # 根据无人机尺寸计算三维安全缓冲，可单独调整每个方向
    safe_margin = [
        (drone_size[0] / 2 + resolution * 0.5) * buffer * buffer_multiplier,  # X轴
        (drone_size[1] / 2 + resolution * 0.5) * buffer * buffer_multiplier,  # Y轴
        (drone_size[2] / 2 + resolution * 0.5) * buffer * buffer_multiplier,  # Z轴
    ]

    grid_shape = (
        int(np.ceil((max_x - min_x) / resolution)),
        int(np.ceil((max_y - min_y) / resolution)),
        int(np.ceil((max_z - min_z) / resolution)),
    )

    # 确保网格是可穿行的，初始都是0（安全）
    grid = np.zeros(grid_shape, dtype=bool)
    print(f"\n初始化碰撞网格 | 分辨率{resolution}m | 安全缓冲{safe_margin}")

    # 障碍物计数，调试用
    obstacle_cell_count = 0

    for idx, (pos, size) in enumerate(obstacles):
        # 计算障碍物实际范围
        x_min = pos[0] - safe_margin[0]
        x_max = pos[0] + size[0] + safe_margin[0]
        y_min = pos[1] - safe_margin[1]
        y_max = pos[1] + size[1] + safe_margin[1]
        z_min = pos[2] - safe_margin[2]
        z_max = pos[2] + size[2] + safe_margin[2]

        # 计算网格中的障碍物范围（确保边界检查）
        gx_min = max(0, int((x_min - min_x) / resolution))
        gx_max = min(grid_shape[0], int((x_max - min_x) / resolution) + 1)
        gy_min = max(0, int((y_min - min_y) / resolution))
        gy_max = min(grid_shape[1], int((y_max - min_y) / resolution) + 1)
        gz_min = max(0, int((z_min - min_z) / resolution))
        gz_max = min(grid_shape[2], int((z_max - min_z) / resolution) + 1)

        # 计算要填充的实际体积
        volume = (gx_max - gx_min) * (gy_max - gy_min) * (gz_max - gz_min)
        obstacle_cell_count += volume

        # 确保范围有效
        if gx_min < gx_max and gy_min < gy_max and gz_min < gz_max:
            grid[gx_min:gx_max, gy_min:gy_max, gz_min:gz_max] = True
            print(
                f"障碍物{idx} | 安全范围 X[{gx_min}-{gx_max}] Y[{gy_min}-{gy_max}] Z[{gz_min}-{gz_max}]"
            )
        else:
            print(f"警告：障碍物{idx}安全范围计算无效，已忽略")
    
    # 验证网格未被完全填充
    total_cells = grid.size
    obstacle_cells = np.sum(grid)
    occupation_ratio = obstacle_cells / total_cells if total_cells > 0 else 0
    
    if occupation_ratio > 0.85:  # 降低到85%发出警告（原90%）
        print(f"警告：障碍物占用率过高({occupation_ratio:.1%})，可能导致无法找到路径")
        
        # 如果障碍物太多，重置网格并增大范围
        if occupation_ratio > 0.9:  # 降低到90%重置（原95%）
            print("占用率过高(>90%)，尝试重新扩大网格空间...")
            # 扩大网格范围，减小安全缓冲区
            new_min_x = min_x - resolution * 10  # 扩大更多空间（原5）
            new_max_x = max_x + resolution * 10
            new_min_y = min_y - resolution * 10
            new_max_y = max_y + resolution * 10
            new_min_z = min_z - resolution * 2  # 增加垂直方向空间
            new_max_z = max_z + resolution * 5  # 上方空间更多
            
            new_grid_shape = (
                int(np.ceil((new_max_x - new_min_x) / resolution)),
                int(np.ceil((new_max_y - new_min_y) / resolution)),
                int(np.ceil((new_max_z - new_min_z) / resolution))
            )
            
            new_grid = np.zeros(new_grid_shape, dtype=bool)
            
            # 重新计算障碍物范围，使用更小的缓冲区
            reduced_safe_margin = [margin * 0.7 for margin in safe_margin]  # 减小到70%
            
            for idx, (pos, size) in enumerate(obstacles):
                x_min = pos[0] - reduced_safe_margin[0]
                x_max = pos[0] + size[0] + reduced_safe_margin[0]
                y_min = pos[1] - reduced_safe_margin[1]
                y_max = pos[1] + size[1] + reduced_safe_margin[1]
                z_min = pos[2] - reduced_safe_margin[2]
                z_max = pos[2] + size[2] + reduced_safe_margin[2]
                
                # 计算新网格中的障碍物范围
                gx_min = max(0, int((x_min - new_min_x) / resolution))
                gx_max = min(new_grid_shape[0], int((x_max - new_min_x) / resolution) + 1)
                gy_min = max(0, int((y_min - new_min_y) / resolution))
                gy_max = min(new_grid_shape[1], int((y_max - new_min_y) / resolution) + 1)
                gz_min = max(0, int((z_min - new_min_z) / resolution))
                gz_max = min(new_grid_shape[2], int((z_max - new_min_z) / resolution) + 1)
                
                # 确保范围有效
                if gx_min < gx_max and gy_min < gy_max and gz_min < gz_max:
                    new_grid[gx_min:gx_max, gy_min:gy_max, gz_min:gz_max] = True
                    print(
                        f"重新计算障碍物{idx} | 安全范围 X[{gx_min}-{gx_max}] Y[{gy_min}-{gy_max}] Z[{gz_min}-{gz_max}]"
                    )
            
            # 验证新网格占用率
            new_occupation_ratio = np.sum(new_grid) / new_grid.size
            print(f"新网格占用率: {new_occupation_ratio:.1%}")
            
            # 使用新网格
            grid = new_grid
            # 更新全局边界信息 (注意: 这不会直接影响bounds参数)
            # 调用者应该意识到这一点
            print(f"网格已重置，新尺寸: {new_grid_shape}")
            
    return grid
