# routers/websocket.py
from fastapi import WebSocket, WebSocketDisconnect
from typing import Dict, List, Optional
import asyncio
import json
from datetime import datetime
from config.settings import DRONE_PHYSICAL_SIZE, GRID_RESOLUTION, UPDATE_INTERVAL
from pathlib import Path
import os

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
async def websocket_endpoint(websocket: WebSocket, task_id: str, path_density: Optional[float] = None):
    """WebSocket端点，处理无人机轨迹模拟
    
    参数:
    - task_id: 任务唯一标识
    - path_density: 可选的路径点密度参数(单位:米)，表示路径点之间的目标距离
    """
    await websocket.accept()
    await drone_trajectory_ws(websocket, task_id, path_density)


async def drone_trajectory_ws(websocket: WebSocket, task_id: str, path_density: Optional[float] = None):
    """处理无人机轨迹WebSocket连接
    
    参数:
    - websocket: WebSocket连接
    - task_id: 任务唯一标识
    - path_density: 可选的路径点密度参数(单位:米)
    """
    try:
        print(f"[INFO] 收到轨迹请求，任务ID: {task_id}，路径密度: {path_density if path_density else '默认'}")
        manager.connect(websocket, task_id)
        
        # 新增：处理WebSocket连接参数
        try:
            # 尝试从WebSocket查询参数获取path_density
            query_params = dict(websocket.query_params)
            if "path_density" in query_params and not path_density:
                try:
                    path_density = float(query_params["path_density"])
                    print(f"[INFO] 从查询参数获取到路径密度: {path_density}米")
                except ValueError:
                    print(f"[WARN] 无效的路径密度值: {query_params['path_density']}")
        except Exception as e:
            print(f"[WARN] 处理WebSocket参数时出错: {str(e)}")

        # 检查有效的路径密度范围
        if path_density is not None:
            if path_density < 0.01:
                path_density = 0.01  # 最小密度，防止点数过多
                print(f"[WARN] 路径密度过小，已调整为: {path_density}米")
            elif path_density > 5.0:
                path_density = 5.0  # 最大密度，防止点数过少
                print(f"[WARN] 路径密度过大，已调整为: {path_density}米")

        # 处理现有任务或创建新任务
        if task_id == "new":
            await handle_ws_error(websocket, "任务不存在")
            return

        # 参数验证
        with task_lock:
            if task_id not in simulation_tasks:
                await handle_ws_error(websocket, "任务不存在")
                return
            task = simulation_tasks[task_id]

        if task.get("status") == "failed":
            await handle_ws_error(websocket, "任务已失败")
            return

        # 智能场景选择 - 根据任务目标位置选择合适的场景
        target_pos = task.get("target_pos", (0, 0, 0))
        
        # 根据坐标特征选择场景
        if target_pos[0] > 0 and target_pos[0] < 10:
            # 测试避障场景
            scene_name = "test.json"
            print(f"[INFO] 根据目标位置选择测试避障场景: {scene_name}")
        else:
            # 默认城市场景
            scene_name = "city_environment.json"
            print(f"[INFO] 根据目标位置选择城市环境场景: {scene_name}")
        
        # 加载场景配置文件
        scene_path = Path(__file__).parent.parent / f"scenarios/presets/{scene_name}"
        
        if not os.path.exists(scene_path):
            print(f"[WARN] 场景文件 {scene_name} 不存在，回退到默认场景")
            scene_path = Path(__file__).parent.parent / "scenarios/presets/city_environment.json"
        
        try:
            with open(scene_path, "r", encoding="utf-8") as f:
                scene_config = json.load(f)
                obstacle_count = len(scene_config.get('obstacles', []))
                print(f"[DEBUG] 成功加载场景文件 {scene_path.name}，包含 {obstacle_count} 个障碍物")
                
                # 验证场景配置有效性
                if obstacle_count == 0:
                    print(f"[WARN] 场景文件 {scene_path.name} 不包含障碍物，请检查配置")
        except Exception as e:
            print(f"[ERROR] 加载场景文件失败: {str(e)}")
            scene_config = {"obstacles": []}
            await handle_ws_error(websocket, f"加载场景文件失败: {str(e)}")
            return

        # 设置路径规划的超时限制
        path_planning_timeout = 20  # 降低到20秒超时限制
        
        try:
            # 使用asyncio.wait_for添加超时控制
            print(f"[DEBUG] 开始路径规划，起点={task['current_pos']}，终点={task['target_pos']}，超时限制: {path_planning_timeout}秒")
            start_time = datetime.now()
            
            # 异步计算路径 - 使用加载的场景配置
            path_params = {
                "current_pos": task["current_pos"],
                "target_pos": task["target_pos"],
                "scene_config": scene_config,
                "drone_size": DRONE_PHYSICAL_SIZE,
            }
            
            # 如果提供了路径密度参数，添加到规划参数中
            if path_density is not None:
                path_params["path_density"] = path_density
                # 更新任务中的路径密度参数
                simulation_tasks[task_id]["path_density"] = path_density
                print(f"[INFO] 使用自定义路径密度: {path_density}米")
            
            raw_path = await asyncio.wait_for(
                asyncio.to_thread(
                    plan_path,
                    **path_params
                ),
                timeout=path_planning_timeout
            )
            
            end_time = datetime.now()
            elapsed = (end_time - start_time).total_seconds()
            print(f"[DEBUG] 路径规划完成，用时: {elapsed:.2f}秒，点数: {len(raw_path)}")
            
            # 检查路径是否为空或点数太少
            if not raw_path or len(raw_path) < 5:  # 降低最小点要求
                raise ValueError(f"路径规划失败：生成的路径点数不足，仅有 {len(raw_path) if raw_path else 0} 个点")
            
        except asyncio.TimeoutError:
            print(f"[ERROR] 路径规划超时，已超过{path_planning_timeout}秒限制")
            await handle_ws_error(websocket, f"路径规划超时，已超过{path_planning_timeout}秒")
            return
        except Exception as e:
            import traceback
            print(f"[ERROR] 路径规划失败: {str(e)}")
            traceback.print_exc()  # 打印完整错误堆栈
            await handle_ws_error(websocket, f"路径规划失败: {str(e)}")
            return

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

        # 推送路径点，设置最大推送时间限制
        max_simulation_time = 180  # 增加最长模拟时间到180秒
        start_time = datetime.now()
        point_batch_size = 5  # 每次一起发送5个点
        point_batch = []
        
        try:
            for idx, point in enumerate(path):
                # 检查是否超过最大模拟时间
                elapsed = (datetime.now() - start_time).total_seconds()
                if elapsed > max_simulation_time:
                    print(f"[WARN] 模拟超过最大时间限制({max_simulation_time}秒)，提前结束")
                    break
                    
                if task_id in simulation_tasks and simulation_tasks[task_id].get("status") == "cancelled":
                    print(f"[INFO] 任务 {task_id} 已被取消")
                    break

                # 添加到当前批次
                point_batch.append(point)
                
                # 更新任务进度
                update_task_progress(task_id, idx)
                
                # 当积累了一批点或达到最后一个点时发送
                if len(point_batch) >= point_batch_size or idx == len(path) - 1:
                    try:
                        # 为每个点添加索引信息
                        for batch_idx, p in enumerate(point_batch):
                            current_idx = idx - len(point_batch) + batch_idx + 1
                            await send_position_update(websocket, current_idx, p, len(path))
                        
                        # 清空批次
                        point_batch = []
                        
                        # 根据时间和进度动态调整延迟
                        progress_ratio = (idx + 1) / len(path)
                        if progress_ratio < 0.2:
                            # 开始阶段快一些
                            delay = UPDATE_INTERVAL * 0.7
                        elif progress_ratio > 0.8:
                            # 结束阶段快一些
                            delay = UPDATE_INTERVAL * 0.7
                        else:
                            # 中间保持正常速度
                            delay = UPDATE_INTERVAL
                            
                        # 确保最小延迟以避免前端过载
                        await asyncio.sleep(max(0.02, delay))
                        
                        # 每50个点打印一次进度
                        if idx % 50 == 0:
                            elapsed_since_start = (datetime.now() - start_time).total_seconds()
                            print(f"[DEBUG] 已推送第 {idx+1}/{len(path)} 个点，耗时: {elapsed_since_start:.2f}秒")
                            
                    except ConnectionError as e:
                        print(f"[ERROR] 连接中断: {str(e)}")
                        raise
                    except Exception as e:
                        print(f"[ERROR] 发送位置更新失败: {str(e)}")
                        # 如果某个批次失败，继续尝试下一个批次
                        point_batch = []

            # 确保发送完成信号
            try:
                # 发送完成信号
                await send_completion(websocket, task_id)
                print(f"[DEBUG] 任务 {task_id} 完成，总推送点数: {len(path)}，总耗时: {(datetime.now() - start_time).total_seconds():.2f}秒")
            except Exception as e:
                print(f"[WARN] 发送完成信号失败: {str(e)}")
                
            # 更新任务状态为已完成
            with task_lock:
                if task_id in simulation_tasks:
                    simulation_tasks[task_id]["status"] = "completed"
                    simulation_tasks[task_id]["completed_at"] = datetime.now().isoformat()
        
        except asyncio.CancelledError:
            print(f"[WARN] 任务 {task_id} 被取消")
            # 更新任务状态为已取消
            with task_lock:
                if task_id in simulation_tasks:
                    simulation_tasks[task_id]["status"] = "cancelled"
            raise  # 重新抛出取消异常
        
        except Exception as e:
            import traceback
            print(f"[ERROR] 推送路径点时出错: {str(e)}")
            traceback.print_exc()  # 打印完整错误堆栈
            
            # 在出错时，尝试更新任务状态
            with task_lock:
                if task_id in simulation_tasks:
                    simulation_tasks[task_id]["status"] = "error"
                    simulation_tasks[task_id]["error"] = f"推送路径点时出错: {str(e)}"
            
            # 尝试发送错误消息
            try:
                await handle_ws_error(websocket, e)
            except:
                pass  # 忽略错误处理中的错误

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
    error_message = str(error)
    await websocket.send_json(
        {"event_type": "ERROR", "error_code": "WS_500", "message": error_message}
    )
    await websocket.close(code=1011)
