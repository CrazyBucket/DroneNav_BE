TRAJECTORY_MIN_POINTS = 50  # 最小路径点数
UPDATE_INTERVAL = 1/60      # 更新间隔（秒）
DRONE_PHYSICAL_SIZE = (0.25, 0.20, 0.06)  # 无人机尺寸
GRID_RESOLUTION = 0.5  # 默认网格分辨率0.5米（平衡精度与计算效率）
OBSTACLE_BUFFER = 1.5  # 障碍物膨胀系数（1.5倍无人机尺寸确保安全距离）