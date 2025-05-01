# services/communication.py
from fastapi import WebSocket
from typing import List, Dict

class DroneConnectionManager:
    """管理 WebSocket 连接"""
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}

    async def connect(self, websocket: WebSocket, task_id: str):
        self.active_connections[task_id] = websocket

    def disconnect(self, websocket: WebSocket):
        for task_id, conn in self.active_connections.items():
            if conn == websocket:
                del self.active_connections[task_id]
                break