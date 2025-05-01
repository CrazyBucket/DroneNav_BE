
# 存储所有模型的基础尺寸（单位：米，未经缩放）
MODEL_DIMENSIONS = {
    # 树木模型
    "tree_1": {"width": 1.7298, "depth": 2.6890, "height": 1.6760},
    
    # 建筑模型（举例，暂时还没模型）
    "office_building_v1": {"width": 15.0, "depth": 10.0, "height": 30.0},
}

def get_model_size(model_id: str) -> dict:
    """安全获取模型尺寸，带异常处理"""
    if model_id not in MODEL_DIMENSIONS:
        raise KeyError(f"未注册的模型ID: {model_id}")
    return MODEL_DIMENSIONS[model_id].copy()