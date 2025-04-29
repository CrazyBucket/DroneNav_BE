# routers/websocket.py
from fastapi import WebSocket, WebSocketDisconnect
from datetime import datetime
import asyncio
import json
from services.communication import DroneConnectionManager
from services.path_planner import plan_path
from main import UPDATE_INTERVAL

manager = DroneConnectionManager()

async def drone_trajectory_ws(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        # 等待初始参数
        init_data = await websocket.receive_json()
        
        # 生成路径规划
        trajectory = await generate_trajectory(init_data)
        
        # 实时推送
        for idx, point in enumerate(trajectory):
            await send_position_update(websocket, idx, point, len(trajectory))
            await asyncio.sleep(UPDATE_INTERVAL)
            
        await send_completion(websocket)
        
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        await handle_ws_error(websocket, e)

async def generate_trajectory(init_data: dict):
    """调用路径规划服务生成轨迹"""
    start = (
        init_data["current_position"]["x"],
        init_data["current_position"]["y"],
        init_data["current_position"]["z"]
    )
    target = (
        init_data["target"]["x"],
        init_data["target"]["y"],
        init_data["target"]["z"]
    )
    
    raw_path = plan_path(
        current_position=start,
        target_position=target,
        obstacles=init_data.get("obstacles", []),
        drone_size=init_data.get("drone_size"),
        grid_resolution=init_data.get("grid_resolution")
    )
    
    return [
        {
            "x": p[0], 
            "y": p[1], 
            "z": p[2],
            "timestamp": datetime.now().timestamp() + i*0.1
        }
        for i, p in enumerate(raw_path)
    ]

async def send_position_update(websocket: WebSocket, idx: int, point: dict, total: int):
    """构造标准化位置更新消息"""
    await websocket.send_json({
        "event_type": "POSITION_UPDATE",
        "data": {
            "sequence": idx + 1,
            "timestamp": datetime.now().isoformat(),
            "coordinates": point,
            "progress": {
                "current": idx + 1,
                "total": total,
                "remaining": total - idx - 1
            }
        }
    })

async def send_completion(websocket: WebSocket):
    """发送任务完成通知"""
    await websocket.send_json({
        "event_type": "MISSION_COMPLETE",
        "data": {
            "timestamp": datetime.now().isoformat(),
            "message": "Trajectory completed"
        }
    })

async def handle_ws_error(websocket: WebSocket, error: Exception):
    """统一错误处理"""
    await websocket.send_json({
        "event_type": "ERROR",
        "error_code": "WS_500",
        "message": str(error)
    })
    await websocket.close(code=1011)