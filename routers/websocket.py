# routers/websocket.py
from fastapi import WebSocket, WebSocketDisconnect
from typing import Dict, List
import asyncio
import json
from datetime import datetime
from config.settings import DRONE_PHYSICAL_SIZE, GRID_RESOLUTION, UPDATE_INTERVAL

from services.communication import DroneConnectionManager
from services.path_planner import plan_path
from models.drone_status import simulation_tasks
from threading import Lock
from uuid import uuid4
from fastapi import APIRouter

router = APIRouter()

manager = DroneConnectionManager()

task_lock = Lock()


@router.websocket("/ws/trajectory/{task_id}")
async def websocket_endpoint(websocket: WebSocket, task_id: str):
    await drone_trajectory_ws(websocket, task_id)


async def drone_trajectory_ws(websocket: WebSocket, task_id: str):
    await websocket.accept()
    await manager.connect(websocket, task_id)
    try:
        print(f"[DEBUG] 开始处理任务 {task_id}")

        # 参数验证
        with task_lock:
            if task_id not in simulation_tasks:
                await handle_ws_error(websocket, "任务不存在")
                return
            task = simulation_tasks[task_id]

        if task.get("status") == "failed":
            await handle_ws_error(websocket, "任务已失败")
            return

        # 异步计算路径
        raw_path = await asyncio.to_thread(
            plan_path,
            current_pos=task["current_pos"],
            target_pos=task["target_pos"],
            scene_config={"obstacles": []},
            drone_size=(0.5, 0.5, 0.5),
        )
        print(f"[DEBUG] 路径规划完成，点数: {len(raw_path)}")

        # 生成带时间戳的路径点
        path = [
            {
                "x": p[0],
                "y": p[1],
                "z": p[2],
                "timestamp": datetime.now().timestamp() + i * UPDATE_INTERVAL,
            }
            for i, p in enumerate(raw_path)
        ]

        # 更新任务状态（加锁）
        with task_lock:
            simulation_tasks[task_id].update(
                {"path": path, "status": "running", "total_points": len(path)}
            )

        # 推送路径点
        for idx, point in enumerate(path):
            if simulation_tasks[task_id].get("status") == "cancelled":
                break

            await send_position_update(websocket, idx, point, len(path))
            update_task_progress(task_id, idx)
            await asyncio.sleep(UPDATE_INTERVAL)
            print(f"[DEBUG] 已推送第 {idx+1}/{len(path)} 个点")

        await send_completion(websocket, task_id)
        print(f"[DEBUG] 任务 {task_id} 完成")

    except Exception as e:
        import traceback

        traceback.print_exc()  # 打印完整错误堆栈
        await handle_ws_error(websocket, e)
    finally:
        manager.disconnect(websocket)


async def send_position_update(websocket: WebSocket, idx: int, point: dict, total: int):
    """构造标准化位置更新消息"""
    await websocket.send_json(
        {
            "event_type": "POSITION_UPDATE",
            "data": {
                "sequence": idx + 1,
                "timestamp": datetime.now().isoformat(),
                "coordinates": point,
                "progress": {
                    "current": idx + 1,
                    "total": total,
                    "remaining": total - idx - 1,
                },
            },
        }
    )


async def send_completion(websocket: WebSocket, task_id: str):
    await websocket.send_json(
        {
            "event_type": "MISSION_COMPLETE",
            "data": {
                "task_id": task_id,  # 添加任务ID
                "timestamp": datetime.now().isoformat(),
                "message": "Trajectory completed",
            },
        }
    )


def update_task_progress(task_id: str, current_step: int):
    """
    更新任务进度状态
    :param task_id: 任务唯一标识
    :param current_step: 当前步骤索引（从0开始）
    """
    if task_id not in simulation_tasks:
        return

    task = simulation_tasks[task_id]
    total_steps = len(task.get("path", []))

    # 计算进度百分比
    progress = (current_step + 1) / total_steps * 100 if total_steps > 0 else 0

    # 更新任务状态
    simulation_tasks[task_id].update(
        {
            "current_step": current_step,
            "progress": round(progress, 2),
            "last_updated": datetime.now().isoformat(),
        }
    )


async def handle_ws_error(websocket: WebSocket, error: Exception):
    """统一错误处理"""
    await websocket.send_json(
        {"event_type": "ERROR", "error_code": "WS_500", "message": str(error)}
    )
    await websocket.close(code=1011)
