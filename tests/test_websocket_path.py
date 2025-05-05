# tests/test_websocket_path.py
import asyncio
import json
import os
import sys
from pathlib import Path

# 添加项目根目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

from services.path_planner import plan_path
from config.settings import DRONE_PHYSICAL_SIZE

async def test_websocket_path_planning():
    """
    测试与websocket.py相同逻辑的路径规划
    """
    print("测试与websocket处理相同的路径规划逻辑")
    
    # 模拟任务参数
    current_pos = (0.0, 0.0, 1.0)
    target_pos = (8.0, 0.0, 1.0)
    
    # 模拟加载场景
    scene_path = Path(project_root) / "scenarios/presets/test.json"
    try:
        with open(scene_path, "r", encoding="utf-8") as f:
            scene_config = json.load(f)
            print(f"成功加载场景文件，包含 {len(scene_config.get('obstacles', []))} 个障碍物")
    except Exception as e:
        print(f"加载场景文件失败: {str(e)}")
        scene_config = {"obstacles": []}
    
    # 计算路径
    print(f"起点: {current_pos}")
    print(f"终点: {target_pos}")
    print(f"无人机尺寸: {DRONE_PHYSICAL_SIZE}")
    
    # 规划路径
    raw_path = plan_path(
        current_pos=current_pos,
        target_pos=target_pos,
        scene_config=scene_config,
        drone_size=DRONE_PHYSICAL_SIZE,
    )
    
    print(f"路径规划完成，点数: {len(raw_path)}")
    
    # 检查路径是否避开了障碍物
    if len(raw_path) <= 2:
        print("❌ 测试失败：路径点太少，无法确定是否绕开障碍物")
        return False
    
    # 计算路径对直线的最大偏离距离
    max_deviation = 0
    direct_line_points = []
    
    # 生成直线路径点进行比较
    steps = 10
    for i in range(steps + 1):
        t = i / steps
        x = current_pos[0] + t * (target_pos[0] - current_pos[0])
        y = current_pos[1] + t * (target_pos[1] - current_pos[1])
        z = current_pos[2] + t * (target_pos[2] - current_pos[2])
        direct_line_points.append((x, y, z))
    
    # 计算每个路径点到直线的最短距离
    for point in raw_path:
        min_dist_to_line = float('inf')
        for i in range(len(direct_line_points) - 1):
            p1 = direct_line_points[i]
            p2 = direct_line_points[i + 1]
            
            # 计算点到线段的距离
            line_vec = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
            point_vec = (point[0] - p1[0], point[1] - p1[1], point[2] - p1[2])
            
            # 计算点在线段上的投影
            line_len = (line_vec[0]**2 + line_vec[1]**2 + line_vec[2]**2)**0.5
            if line_len == 0:
                continue
                
            proj = (point_vec[0]*line_vec[0] + point_vec[1]*line_vec[1] + point_vec[2]*line_vec[2]) / line_len
            
            if 0 <= proj <= line_len:
                # 点到线段的距离
                dist = ((point_vec[0] - proj*line_vec[0]/line_len)**2 + 
                        (point_vec[1] - proj*line_vec[1]/line_len)**2 + 
                        (point_vec[2] - proj*line_vec[2]/line_len)**2)**0.5
                min_dist_to_line = min(min_dist_to_line, dist)
            
        if min_dist_to_line < float('inf'):
            max_deviation = max(max_deviation, min_dist_to_line)
    
    print(f"路径最大偏离直线距离: {max_deviation:.2f}米")
    
    # 如果最大偏离距离大于阈值，认为避障有效
    if max_deviation > 2.0:  # 设置2米的阈值
        print("✅ 测试通过：路径成功避开了障碍物！")
        # 打印部分路径点
        sample_interval = max(1, len(raw_path) // 10)
        print("路径点示例:")
        for i in range(0, len(raw_path), sample_interval):
            print(f"点 {i}: {raw_path[i]}")
        return True
    else:
        print("❌ 测试失败：路径似乎没有显著避开障碍物")
        return False

if __name__ == "__main__":
    asyncio.run(test_websocket_path_planning()) 