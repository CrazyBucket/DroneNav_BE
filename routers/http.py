from uuid import uuid4
from fastapi import APIRouter, BackgroundTasks, Query
import json
from pathlib import Path
import asyncio
import os

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
    scene_id: str = None  # 添加场景ID字段，默认为None

@router.post("/start_simulation")
async def start_simulation(request: SimulationRequest):
    task_id = str(uuid4())
    
    # 打印接收到的请求参数
    scene_id = request.scene_id
    current_pos = (request.current.x, request.current.y, request.current.z)
    target_pos = (request.target.x, request.target.y, request.target.z)
    print(f"[INFO] 接收到仿真请求: 起点={current_pos}, 终点={target_pos}, 场景ID={scene_id or '未提供'}")
    
    # 存储任务信息
    simulation_tasks[task_id] = {
        "current_pos": current_pos,
        "target_pos": target_pos,
        "status": "pending",
        "scene_id": scene_id  # 存储场景ID
    }
    
    print(f"[INFO] 创建仿真任务: ID={task_id}, 场景ID={scene_id or '默认'}")
    
    return {
        "status": "started",
        "task_id": task_id,
        "ws_endpoint": f"/ws/trajectory/{task_id}"
    }
@router.options("/{path:path}")
async def options_handler():
    return {"status": "ok"}

@router.get("/get_scenes")
async def get_scene_list():
    """获取所有可用场景的列表"""
    scenarios_dir = Path(__file__).parent.parent / "scenarios/presets"
    scene_files = [f for f in os.listdir(scenarios_dir) if f.endswith('.json')]
    
    scenes = []
    for file_name in scene_files:
        try:
            with open(scenarios_dir / file_name, "r", encoding="utf-8") as f:
                scene_data = json.load(f)
                scenes.append({
                    "id": file_name.replace('.json', ''),
                    "name": scene_data.get("name", file_name),
                    "description": scene_data.get("description", "无描述"),
                    "object_count": len(scene_data.get("obstacles", [])),
                })
        except Exception as e:
            print(f"读取场景文件 {file_name} 出错: {str(e)}")
    
    return {
        "status": "success",
        "scenes": scenes
    }

@router.get("/getScene")
async def get_scene(scene_id: str = Query(None, description="场景ID，不提供则使用默认场景")):
    """获取特定场景配置，如果不指定场景ID则返回默认场景"""
    if not scene_id:
        # 默认场景
        scene_id = "city_environment"
    
    scene_path = Path(__file__).parent.parent / f"scenarios/presets/{scene_id}.json"
    
    if not scene_path.exists():
        return {
            "status": "error",
            "message": f"场景不存在: {scene_id}"
        }
    
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

async def plan_path(current_pos, target_pos, scene_config, drone_size):
    # This function is assumed to exist and be imported from a module
    # It's called with the parameters from the task
    # The implementation of this function is not provided in the original file or the code block
    # It's assumed to exist and return a path
    pass

@router.post("/plan_path")
async def plan_path_endpoint(task_id: str):
    if task_id not in simulation_tasks:
        return {"status": "error", "message": "任务不存在"}

    task = simulation_tasks[task_id]
    # 创建测试场景
    scene_config = create_test_scenario()
    path = plan_path(task["current_pos"], task["target_pos"], scene_config, (0.5, 0.5, 0.5))

    return {
        "status": "success",
        "raw_path": path
    }