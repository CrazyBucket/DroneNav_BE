import hashlib
import hmac
import time
from typing import Dict, Optional, List, Any
from fastapi import Request, HTTPException, status
import re

# 存储设备指纹信息
device_fingerprints: Dict[str, List[Dict[str, Any]]] = {}

# 可疑设备指纹记录
suspicious_fingerprints: List[Dict[str, Any]] = []

# 最大允许的设备数量（每个用户）
MAX_DEVICES_PER_USER = 5

# 指纹相似度阈值
FINGERPRINT_SIMILARITY_THRESHOLD = 0.85

def calculate_fingerprint_similarity(fp1: str, fp2: str) -> float:
    """
    计算两个指纹的相似度
    
    Args:
        fp1: 第一个指纹
        fp2: 第二个指纹
        
    Returns:
        float: 相似度，范围0-1
    """
    # 简单实现，实际应用中可以使用更复杂的算法
    if not fp1 or not fp2:
        return 0.0
    
    # 将指纹拆分为组件
    components1 = fp1.split('|')
    components2 = fp2.split('|')
    
    if len(components1) != len(components2):
        return 0.0
    
    # 计算匹配的组件数量
    matches = sum(1 for c1, c2 in zip(components1, components2) if c1 == c2)
    return matches / len(components1)

def validate_fingerprint(user_id: str, fingerprint: str, ip_address: str) -> bool:
    """
    验证设备指纹
    
    Args:
        user_id: 用户ID
        fingerprint: 设备指纹
        ip_address: IP地址
        
    Returns:
        bool: 指纹是否有效
    """
    # 如果用户没有记录，则创建新记录
    if user_id not in device_fingerprints:
        device_fingerprints[user_id] = []
    
    # 检查是否为已知设备
    user_devices = device_fingerprints[user_id]
    current_time = time.time()
    
    # 检查是否与现有设备匹配
    for device in user_devices:
        similarity = calculate_fingerprint_similarity(device["fingerprint"], fingerprint)
        if similarity >= FINGERPRINT_SIMILARITY_THRESHOLD:
            # 更新最后使用时间和IP
            device["last_seen"] = current_time
            device["ip_address"] = ip_address
            return True
    
    # 如果是新设备，检查用户设备数量是否超出限制
    if len(user_devices) >= MAX_DEVICES_PER_USER:
        # 记录可疑行为
        suspicious_fingerprints.append({
            "user_id": user_id,
            "fingerprint": fingerprint,
            "ip_address": ip_address,
            "timestamp": current_time,
            "reason": "设备数量超出限制"
        })
        return False
    
    # 添加新设备
    user_devices.append({
        "fingerprint": fingerprint,
        "first_seen": current_time,
        "last_seen": current_time,
        "ip_address": ip_address
    })
    
    return True

def extract_fingerprint(request: Request) -> Optional[str]:
    """
    从请求头中提取设备指纹
    
    Args:
        request: FastAPI请求对象
        
    Returns:
        Optional[str]: 设备指纹，如果不存在则为None
    """
    return request.headers.get("X-Device-Fingerprint")

def extract_client_ip(request: Request) -> str:
    """
    从请求中提取客户端IP地址
    
    Args:
        request: FastAPI请求对象
        
    Returns:
        str: 客户端IP地址
    """
    forwarded = request.headers.get("X-Forwarded-For")
    if forwarded:
        return forwarded.split(",")[0].strip()
    return request.client.host if request.client else "unknown"

def is_valid_fingerprint(fingerprint: str) -> bool:
    """
    验证指纹格式是否有效
    
    Args:
        fingerprint: 设备指纹
        
    Returns:
        bool: 指纹格式是否有效
    """
    if not fingerprint:
        return False
    
    # 检查指纹格式
    # 格式示例: canvas-hash|screen-resolution|timezone|platform|webgl-hash
    pattern = r'^[a-f0-9]{32}\|[0-9]+x[0-9]+\|[+-][0-9]+\|[a-zA-Z0-9 ]+\|[a-f0-9]{32}$'
    return bool(re.match(pattern, fingerprint))

async def verify_device_fingerprint(request: Request, user_id: str) -> None:
    """
    验证设备指纹，如果无效则抛出异常
    
    Args:
        request: FastAPI请求对象
        user_id: 用户ID
        
    Raises:
        HTTPException: 如果指纹无效
    """
    fingerprint = extract_fingerprint(request)
    if not fingerprint:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="缺少设备指纹"
        )
    
    if not is_valid_fingerprint(fingerprint):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="无效的设备指纹格式"
        )
    
    ip_address = extract_client_ip(request)
    if not validate_fingerprint(user_id, fingerprint, ip_address):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="设备未授权，请联系管理员"
        )

def get_user_devices(user_id: str) -> List[Dict[str, Any]]:
    """
    获取用户的设备列表
    
    Args:
        user_id: 用户ID
        
    Returns:
        List[Dict[str, Any]]: 设备列表
    """
    return device_fingerprints.get(user_id, []) 