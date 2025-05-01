import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from datetime import datetime
import uvicorn
from routers.websocket import router as websocket_router
from starlette.middleware.cors import CORSMiddleware
from starlette.testclient import TestClient
from routers.http import router as http_router

app = FastAPI()

app.include_router(websocket_router)
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

if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8001,
        ws='websockets',
        reload=True,
        headers=[("Server", "DroneControl/1.0.0")]  # 添加自定义响应头
    )