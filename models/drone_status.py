from typing import Dict, Any

drone_status = {
    "current_position": {"x": 0.0, "y": 0.0, "z": 0.0},
}

# 存储所有仿真任务
simulation_tasks: Dict[str, Dict[str, Any]] = {}