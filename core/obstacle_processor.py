from pydantic import BaseModel
from typing import Dict, List, Tuple
from config.model_registry import get_model_size

def parse_obstacles(scene_config: Dict) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """将场景配置中的障碍物转换为标准格式"""
    standardized = []
    
    for obs in scene_config["obstacles"]:
        try:
            # 根据不同类型解析尺寸
            if obs["type"] == "TREE":
                model_id = obs["feature"]["model"]
                scale = obs["feature"]["scale"]
                # 这里应该调用模型尺寸数据库
                base_size = get_model_size(model_id)  # 示例函数
                actual_size = (
                    base_size[0] * scale,
                    base_size[1] * scale,
                    base_size[2] * scale
                )
                
            elif obs["type"] == "BUILDING":
                footprint = obs["feature"]["footprint"]
                actual_size = (
                    footprint[0],
                    footprint[1],
                    obs["feature"]["height"]
                )
                
            # 添加更多障碍物类型处理...
            
            # 获取位置
            pos = (
                obs["position"]["x"],
                obs["position"]["y"],
                obs["position"]["z"]
            )
            
            standardized.append((pos, actual_size))
            
        except KeyError as e:
            print(f"Invalid obstacle config: {e}")
            continue
            
    return standardized

def process_scene_obstacles(obstacles: List[BaseModel]) -> list:
    """批量处理场景中的所有障碍物"""
    return [parse_obstacles(obs) for obs in obstacles]