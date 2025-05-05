# core/obstacle_processor.py
from pydantic import BaseModel
from typing import Dict, List, Tuple
from config.model_registry import get_model_size

def parse_obstacles(scene_config: Dict) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    standardized = []
    
    # 检查场景配置是否有效
    if not scene_config or "obstacles" not in scene_config or not scene_config["obstacles"]:
        print("警告: 场景中没有障碍物!")
        return []
    
    # 跟踪已处理的障碍物类型
    processed_counts = {"BUILDING": 0, "TREE": 0, "DRONE": 0, "CUBE": 0, "OTHER": 0}
    
    for obs in scene_config["obstacles"]:
        try:
            pos = None
            actual_size = None
            
            if obs["type"] == "BUILDING":
                footprint = obs["feature"]["footprint"]
                height = obs["feature"]["height"]
                
                # 中心点坐标
                center_x = obs["position"]["x"]
                center_y = obs["position"]["y"]
                center_z = obs["position"]["z"]
                
                # 建筑物总尺寸
                actual_size = (
                    float(footprint[0]),  # X轴宽度
                    float(footprint[1]),  # Y轴深度
                    float(height)         # Z轴高度
                )
                
                # 计算左下角坐标
                pos = (
                    center_x - actual_size[0]/2,  # 左下角X坐标
                    center_y - actual_size[1]/2,  # 左下角Y坐标
                    center_z                      # 底部Z坐标
                )
                processed_counts["BUILDING"] += 1
                
            elif obs["type"] == "TREE":
                try:
                    model_id = obs["feature"]["model"]
                    scale = obs["feature"]["scale"]
                    base_size = get_model_size(model_id)
                    
                    # 树的三维尺寸
                    actual_size = (
                        base_size['width'] * scale,
                        base_size['depth'] * scale,
                        base_size['height'] * scale
                    )
                    
                    # 树的位置（保持中心点）
                    pos = (
                        obs["position"]["x"],
                        obs["position"]["y"],
                        obs["position"]["z"]
                    )
                    processed_counts["TREE"] += 1
                except KeyError as e:
                    print(f"Invalid tree model config: {e}")
                    continue
                
            elif obs["type"] == "DRONE":
                drone_id = obs.get("id", "unknown")
                model = obs.get("model", "DEFAULT")
                
                # 获取无人机型号尺寸
                drone_size = get_model_size(model)
                
                # 无人机当前位置（中心点）
                center_x = obs["position"]["x"]
                center_y = obs["position"]["y"]
                center_z = obs["position"]["z"]
                
                # 计算左下角坐标
                pos = (
                    center_x - drone_size[0]/2,  # 左下角X坐标
                    center_y - drone_size[1]/2,  # 左下角Y坐标
                    center_z - drone_size[2]/2   # 左下角Z坐标
                )
                
                actual_size = drone_size
                processed_counts["DRONE"] += 1
                
            elif obs["type"] == "CUBE":
                try:
                    # 立方体的尺寸
                    cube_size = obs["feature"]["size"]
                    
                    # 立方体的中心坐标
                    center_x = obs["position"]["x"]
                    center_y = obs["position"]["y"]
                    center_z = obs["position"]["z"]
                    
                    # 立方体的实际尺寸
                    actual_size = (
                        float(cube_size[0]),  # X轴宽度
                        float(cube_size[1]),  # Y轴深度
                        float(cube_size[2])   # Z轴高度
                    )
                    
                    # 计算左下角坐标（而不是使用中心点作为位置）
                    pos = (
                        center_x - actual_size[0]/2,  # 左下角X坐标
                        center_y - actual_size[1]/2,  # 左下角Y坐标
                        center_z - actual_size[2]/2   # 左下角Z坐标
                    )
                    processed_counts["CUBE"] += 1
                    
                except Exception as e:
                    print(f"处理CUBE类型障碍物时出错: {e}")
                    continue
            else:
                processed_counts["OTHER"] += 1
                print(f"跳过未知障碍物类型: {obs['type']}")
                continue
                
            # 只有当位置和尺寸都正确计算时才添加
            if pos is not None and actual_size is not None:
                standardized.append((pos, actual_size))
                
        except KeyError as e:
            print(f"Invalid obstacle config: {e}")
            continue
            
    if not standardized:
        print("警告: 未找到有效的障碍物!")
    else:
        print(f"障碍物处理统计: BUILDING={processed_counts['BUILDING']}, TREE={processed_counts['TREE']}, "
              f"DRONE={processed_counts['DRONE']}, CUBE={processed_counts['CUBE']}, 其他={processed_counts['OTHER']}")
        print(f"成功处理 {len(standardized)} 个障碍物")
        
    return standardized
