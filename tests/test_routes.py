# 临时测试脚本
from services.path_planner import plan_path

def test_plan_path():
    start = (0, 0, 0)
    goal = (10, 10, 5)
    drone_size = (0.5, 0.5, 0.5)  # 确保与DRONE_PHYSICAL_SIZE一致
    grid_res = 0.5
    
    path = plan_path(
        current_position=start,
        target_position=goal,
        drone_size=drone_size,
        grid_resolution=grid_res
    )
    
    print(f"路径点数: {len(path)}")
    print("示例路径点:", path[:3])  # 打印前3个点

test_plan_path()