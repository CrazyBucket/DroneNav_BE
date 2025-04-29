from fastapi import APIRouter
from typing import Dict
from datetime import datetime
import json
from pathlib import Path
from services.path_planner import plan_path
from models.drone_status import drone_status

router = APIRouter()

@router.post("/start_simulation")
async def start_simulation(coordinates: Dict[str, float]):
    """
    接收前端发送的坐标并启动仿真
    - 参数: {"x": float, "y": float, "z": float}
    - 返回: 确认信息和当前状态
    """
    # 更新目标位置
    drone_status["target"] = coordinates
    
    # 生成路径
    full_path = plan_path(
        drone_status["current_position"],
        drone_status["target"],
        drone_status["speed"]
    )
    drone_status["path"] = full_path
    
    return {
        "status": "success",
        "message": "仿真已启动",
        "target_position": drone_status["target"],
        "path_points": len(full_path)
    }

@router.options("/{path:path}")
async def options_handler():
    return {"status": "ok"}

@router.get("/test")
async def test_interface():
    """测试接口，返回基本状态信息"""
    return {
        "status": "success",
        "timestamp": datetime.now().isoformat(),
        "service_info": {
            "service": "Drone Navigation Service",
            "version": "1.0.0",
            "status": "running",
            "drone_status": {
                "current_position": drone_status["current_position"],
                "target_position": drone_status["target"],
                "speed": drone_status["speed"],
                "altitude": drone_status["altitude"],
                "path_points": len(drone_status["path"])
            }
        }
    }

@router.get("/getScene")
async def get_scene():
    """获取城市场景配置"""
    scene_path = Path(__file__).parent.parent / "scenarios/presets/city_environment.json"
    try:
        with open(scene_path, "r", encoding="utf-8") as f:
            scene_data = json.load(f)
        return {
            "status": "success",
            "scene": scene_data
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"无法读取场景文件: {str(e)}"
        }