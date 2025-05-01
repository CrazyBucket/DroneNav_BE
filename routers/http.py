from uuid import uuid4
from fastapi import APIRouter, BackgroundTasks
import json
from pathlib import Path

from pydantic import BaseModel
from models.drone_status import drone_status
from models.drone_status import simulation_tasks


router = APIRouter()
class Coordinate(BaseModel):
    x: float
    y: float
    z: float

class SimulationRequest(BaseModel):
    current: Coordinate
    target: Coordinate

@router.post("/start_simulation")
async def start_simulation(request: SimulationRequest):
    task_id = str(uuid4())
    simulation_tasks[task_id] = {
        "current_pos": (request.current.x, request.current.y, request.current.z),
        "target_pos": (request.target.x, request.target.y, request.target.z),
        "status": "pending"
    }
    return {
        "status": "started",
        "task_id": task_id,
        "ws_endpoint": f"/ws/trajectory/{task_id}"
    }
@router.options("/{path:path}")
async def options_handler():
    return {"status": "ok"}

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