import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from datetime import datetime
import uvicorn
import threading
import time

from starlette.middleware.cors import CORSMiddleware
from starlette.testclient import TestClient

from services.path_planner import plan_path
from routers.http import router as http_router
from models.drone_status import drone_status


TRAJECTORY_MIN_POINTS = 50  # 最小路径点数
UPDATE_INTERVAL = 0.1       # 更新间隔（秒）
DRONE_PHYSICAL_SIZE = (0.25, 0.20, 0.06)  # 无人机尺寸
GRID_RESOLUTION = 0.5  # 默认网格分辨率0.5米（平衡精度与计算效率）
OBSTACLE_BUFFER = 1.5  # 障碍物膨胀系数（1.5倍无人机尺寸确保安全距离）

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=[
        "x-request-time",  # 显式允许自定义头
        "content-type",    # 常规头仍需列出
        "authorization",
        "*"                # 保留通配符确保兼容
    ],
)

app.include_router(http_router)
# 无人机状态存储
drone_status = {
    "current_position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "target": {"x": 10.0, "y": 10.0, "z": 5.0},
    "speed": 5.0,  # m/s
    "altitude": 5.0,
    "path": []
}


def calculate_trajectory(start, target, speed):
    """使用路径规划算法生成路径"""
    # 转换坐标格式为元组
    start_pos = (start["x"], start["y"], start["z"])
    target_pos = (target["x"], target["y"], target["z"])

    # 调用路径规划算法（假设没有障碍物，使用默认参数）
    path = plan_path(
        current_position=start_pos,
        target_position=target_pos,
        obstacles=[],
        drone_size=DRONE_PHYSICAL_SIZE,
        grid_resolution=GRID_RESOLUTION
    )

    # 计算总时间（根据速度和距离）
    total_distance = sum(
        ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2) ** 0.5
        for p1, p2 in zip(path[:-1], path[1:])
    )
    time_step = total_distance / (speed * len(path)) if len(path) > 0 else 0.1

    # 添加时间戳信息
    return [
        {
            "x": point[0],
            "y": point[1],
            "z": point[2],
            "timestamp": time.time() + i * time_step
        }
        for i, point in enumerate(path)
    ]


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        # 接收初始参数
        init_data = await websocket.receive_json()
        drone_status.update({
            "current_position": init_data["current_position"],
            "target": init_data["target"],
            "speed": init_data["speed"],
            "altitude": init_data["altitude"]
        })

        # 生成完整路径
        full_path = calculate_trajectory(
            drone_status["current_position"],
            drone_status["target"],
            drone_status["speed"]
        )
        drone_status["path"] = full_path

        # 实时发送位置更新
        for idx, point in enumerate(full_path):
            response = {
                "timestamp": datetime.now().isoformat(),
                "current_position": point,
                "path_index": idx,
                "total_points": len(full_path),
                "remaining_distance": (len(full_path) - idx) * drone_status["speed"] * 0.1
            }
            await websocket.send_json(response)
            await asyncio.sleep(0.1)  # 100ms更新一次

        # 到达目标后发送完成通知
        await websocket.send_json({
            "status": "complete",
            "message": "目标位置已到达"
        })

    except WebSocketDisconnect:
        print("客户端断开连接")
    except Exception as e:
        print(f"通信错误: {str(e)}")


def simulate_client():
    """改进的客户端模拟"""
    with TestClient(app).websocket_connect("/ws") as ws:
        # 发送初始参数
        ws.send_json({
            "current_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "target": {"x": 56.0, "y": 100.0, "z": 26.0},
            "speed": 10.0,  # 10 m/s
            "altitude": 50.0
        })

        try:
            while True:
                data = ws.receive_json()
                if data.get("status") == "complete":
                    print("\n[任务完成] 无人机已到达目标位置")
                    break

                print(f"\n时间: {data['timestamp']}")
                print(f"当前位置: X:{data['current_position']['x']:.2f} "
                      f"Y:{data['current_position']['y']:.2f} "
                      f"Z:{data['current_position']['z']:.2f}")
                print(f"进度: {data['path_index']}/{data['total_points']} "
                      f"剩余距离: {data['remaining_distance']:.2f}米")

        except Exception as e:
            print(f"客户端错误: {str(e)}")


if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8001,
        ws='websockets',
        reload=True,
        headers=[("Server", "DroneControl/1.0.0")]  # 添加自定义响应头
    )