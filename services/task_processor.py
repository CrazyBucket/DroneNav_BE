# services/task_processor.py
import asyncio
import json
import os
import numpy as np
from services.path_planner import plan_path
from models.drone_status import simulation_tasks

# 加载默认城市环境
def load_default_environment():
    """加载默认的城市环境配置"""
    try:
        # env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
        #                       "scenarios", "presets", "test.json")
        env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                              "scenarios", "presets", "city_environment.json")
        with open(env_path, 'r', encoding='utf-8') as f:
            environment = json.load(f)
        print(f"成功加载默认环境：{environment['name']}")
        print(f"环境中包含{len(environment['obstacles'])}个障碍物")
        return environment
    except Exception as e:
        print(f"加载默认环境失败: {str(e)}")
        return {"obstacles": []}  # 返回空环境作为后备

# 全局变量，存储默认环境
DEFAULT_ENVIRONMENT = load_default_environment()

async def process_simulation_task(task_id: str):
    """唯一负责路径规划计算的函数"""
    task = simulation_tasks[task_id]
    try:
        # 优先使用任务中指定的场景配置，如果没有则使用默认环境
        scene_config = task.get("scene_config", DEFAULT_ENVIRONMENT)
        
        # 确保场景配置有obstacles字段
        if "obstacles" not in scene_config:
            scene_config["obstacles"] = DEFAULT_ENVIRONMENT.get("obstacles", [])
        
        # 打印任务配置信息
        print(f"\n处理飞行任务 ID: {task_id}")
        print(f"起点: {task['current_pos']}")
        print(f"终点: {task['target_pos']}")
        print(f"障碍物数量: {len(scene_config['obstacles'])}")
        
        # 从任务中获取路径密度参数，默认为0.3米（更合理的间距）
        path_density = task.get("path_density", 0.3)
        
        # 根据路径长度动态调整默认密度
        if path_density == 0.3:  # 如果是默认值，进行动态调整
            # 计算起点和终点之间的直线距离
            start_pos = np.array(task["current_pos"])
            end_pos = np.array(task["target_pos"])
            distance = np.linalg.norm(end_pos - start_pos)
            
            # 根据距离动态调整默认密度
            if distance < 5:  # 短距离，使用较密的点
                path_density = 0.2
            elif distance < 20:  # 中等距离
                path_density = 0.3 
            elif distance < 50:  # 较长距离
                path_density = 0.5
            else:  # 超长距离
                path_density = 1.0
                
            print(f"根据路径长度({distance:.2f}米)动态调整密度为: {path_density}米")
            
        print(f"使用路径点密度: {path_density}米")
        
        path = await asyncio.to_thread(
            plan_path,
            current_pos=task["current_pos"],
            target_pos=task["target_pos"],
            scene_config=scene_config,  # 使用完整的场景配置
            drone_size=(0.5, 0.5, 0.5),
            grid_resolution=0.5,  # 设置合理的网格分辨率
            path_density=path_density  # 传递路径密度参数
        )
        
        # 更新任务状态
        point_count = len(path)
        simulation_tasks[task_id].update({
            "path": path,
            "status": "running" if point_count > 0 else "failed",
            "total_points": point_count,
            "message": f"规划了{point_count}个路径点" if point_count > 0 else "无法找到有效路径"
        })
        
        print(f"路径规划完成，共{point_count}个点")
        
    except Exception as e:
        print(f"路径规划失败: {str(e)}")
        import traceback
        traceback.print_exc()  # 打印详细错误信息
        simulation_tasks[task_id].update({
            "status": "failed",
            "error": str(e),
            "message": f"规划失败: {str(e)}"
        })