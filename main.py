import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from services.path_planner import plan_path
import rospy

# 创建 FastAPI 实例
app = FastAPI()

# 定义请求体模型
class TargetCoordinates(BaseModel):
    target: dict  # {"x": float, "y": float, "z": float}
    speed: float
    altitude: float
    current_position: dict  # 当前的位置：{"x": float, "y": float, "z": float}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()  # 接受连接
    try:
        while True:
            # 接收来自前端的目标位置、当前状态等信息
            data = await websocket.receive_json()
            target = data.get("target")
            current_position = data.get("current_position")
            speed = data.get("speed")
            altitude = data.get("altitude")
            
            # 调用路径规划函数，计算飞行路径
            path_planning_result = plan_path(current_position, target)
            
            if path_planning_result:
                # 假设返回的路径是一个简化的示例路径
                trajectory = [
                    {"x": 12.00, "y": 56.00, "z": 10.0},
                    {"x": 12.10, "y": 56.10, "z": 10.0},
                    {"x": 12.20, "y": 56.20, "z": 10.0},
                ]
                response = {
                    "status": "success",
                    "current_position": current_position,
                    "path": trajectory,
                    "remaining_distance": 20.5
                }
                # 发送轨迹给前端
                await websocket.send_json(response)
            else:
                # 如果路径规划失败
                await websocket.send_json({"status": "failure", "message": "Path planning failed"})
                
            # 假设每2秒更新一次状态
            await asyncio.sleep(2)
    
    except WebSocketDisconnect:
        print("Client disconnected")

@app.get("/status")
async def get_status():
    return {"status": "success", "message": "无人机正在飞行中"}
