from fastapi import APIRouter, Depends, HTTPException, status, Body
from fastapi.security import OAuth2PasswordRequestForm
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, EmailStr, Field, validator
import re
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import jwt
import secrets

from models.user import (
    User, create_user, get_user_by_username, get_user_by_email, get_user_by_id,
    users_db, hash_password
)
from core.auth import (
    create_access_token, create_refresh_token, verify_refresh_token, decode_token,
    get_current_user_id, ACCESS_TOKEN_EXPIRE_MINUTES
)

# 创建路由器
router = APIRouter(prefix="/api/auth", tags=["认证"])

# 定义请求和响应模型
class UserCreate(BaseModel):
    username: str = Field(..., min_length=3, max_length=20)
    email: EmailStr
    password: str = Field(..., min_length=8)
    
    @validator('password')
    def password_complexity(cls, v):
        """验证密码复杂度"""
        if not re.search(r'[A-Z]', v):
            raise ValueError('密码必须包含至少一个大写字母')
        if not re.search(r'[a-z]', v):
            raise ValueError('密码必须包含至少一个小写字母')
        if not re.search(r'[0-9]', v):
            raise ValueError('密码必须包含至少一个数字')
        if not re.search(r'[^A-Za-z0-9]', v):
            raise ValueError('密码必须包含至少一个特殊字符')
        return v

class UserLogin(BaseModel):
    username: str
    password: str

class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int = ACCESS_TOKEN_EXPIRE_MINUTES * 60  # 转换为秒

class RefreshRequest(BaseModel):
    refresh_token: str

class VerificationResponse(BaseModel):
    message: str
    temp_token: Optional[str] = None
    exists: bool = True

class PasswordResetRequest(BaseModel):
    email: EmailStr

class PasswordResetConfirm(BaseModel):
    temp_token: str
    verification_code: str
    new_password: str = Field(..., min_length=8)
    
    @validator('new_password')
    def password_complexity(cls, v):
        """验证密码复杂度"""
        if not re.search(r'[A-Z]', v):
            raise ValueError('密码必须包含至少一个大写字母')
        if not re.search(r'[a-z]', v):
            raise ValueError('密码必须包含至少一个小写字母')
        if not re.search(r'[0-9]', v):
            raise ValueError('密码必须包含至少一个数字')
        if not re.search(r'[^A-Za-z0-9]', v):
            raise ValueError('密码必须包含至少一个特殊字符')
        return v

class UserResponse(BaseModel):
    user_id: str
    username: str
    email: str
    role: str
    last_login: Optional[str] = None

# 模拟密码重置令牌存储
password_reset_tokens = {}

# 模拟验证码存储
verification_codes = {}

@router.post("/register", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def register(user_data: UserCreate):
    """
    注册新用户
    """
    try:
        user = create_user(
            username=user_data.username,
            email=user_data.email,
            password=user_data.password
        )
        return user.to_dict()
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

@router.post("/login", response_model=TokenResponse)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    """
    用户登录
    """
    user = get_user_by_username(form_data.username)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="用户名或密码不正确",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    if not user.verify_password(form_data.password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="用户名或密码不正确",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # 更新最后登录时间
    user.update_last_login()
    users_db[user.user_id] = user.to_dict(include_sensitive=True)
    
    # 创建访问令牌和刷新令牌
    token_data = {"sub": user.user_id}
    access_token = create_access_token(token_data)
    refresh_token = create_refresh_token(token_data)
    
    return {
        "access_token": access_token,
        "refresh_token": refresh_token,
        "token_type": "bearer",
        "expires_in": ACCESS_TOKEN_EXPIRE_MINUTES * 60
    }

@router.post("/refresh", response_model=TokenResponse)
async def refresh_token(refresh_req: RefreshRequest):
    """
    使用刷新令牌获取新的访问令牌
    """
    try:
        payload = verify_refresh_token(refresh_req.refresh_token)
        user_id = payload.get("sub")
        
        if not user_id or not get_user_by_id(user_id):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="无效的刷新令牌",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        # 创建新的访问令牌
        token_data = {"sub": user_id}
        access_token = create_access_token(token_data)
        
        return {
            "access_token": access_token,
            "refresh_token": refresh_req.refresh_token,  # 返回相同的刷新令牌
            "token_type": "bearer",
            "expires_in": ACCESS_TOKEN_EXPIRE_MINUTES * 60
        }
    except HTTPException:
        raise
    except Exception:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="无效的刷新令牌",
            headers={"WWW-Authenticate": "Bearer"},
        )

@router.post("/password-reset/request", response_model=VerificationResponse)
async def request_password_reset(req: PasswordResetRequest):
    """
    请求密码重置，生成验证码
    """
    user = get_user_by_email(req.email)
    if not user:
        return VerificationResponse(
            message="该邮箱未注册",
            exists=False
        )
    
    # 生成6位数字验证码
    verification_code = ''.join([str(secrets.randbelow(10)) for _ in range(6)])
    
    # 生成临时令牌
    temp_token = create_access_token(
        {"sub": user.user_id, "purpose": "password_reset"},
        expires_delta=timedelta(minutes=10)
    )
    
    # 存储验证信息
    verification_codes[temp_token] = {
        "code": verification_code,
        "user_id": user.user_id,
        "expires_at": datetime.utcnow() + timedelta(minutes=10)
    }
    
    # 在实际应用中，这里应该发送验证码到用户邮箱
    # 这里为了测试，直接返回验证码
    return VerificationResponse(
        message=f"验证码已发送到邮箱（测试环境验证码：{verification_code}）",
        temp_token=temp_token,
        exists=True
    )

@router.post("/password-reset/confirm")
async def confirm_password_reset(req: PasswordResetConfirm):
    """
    验证验证码并重置密码
    """
    try:
        # 获取验证信息
        verification_info = verification_codes.get(req.temp_token)
        if not verification_info:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="无效的临时令牌"
            )
        
        # 检查是否过期
        if datetime.utcnow() > verification_info["expires_at"]:
            del verification_codes[req.temp_token]
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="验证码已过期"
            )
        
        # 验证验证码
        if req.verification_code != verification_info["code"]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="验证码错误"
            )
        
        # 获取用户并更新密码
        user = get_user_by_id(verification_info["user_id"])
        if not user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="用户不存在"
            )
        
        # 更新密码
        user.password_hash, user.salt = hash_password(req.new_password)
        users_db[user.user_id] = user.to_dict(include_sensitive=True)
        
        # 清理验证信息
        del verification_codes[req.temp_token]
        
        return {"message": "密码已成功重置"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"密码重置失败: {str(e)}"
        )

@router.get("/me", response_model=UserResponse)
async def get_current_user(user_id: str = Depends(get_current_user_id)):
    """
    获取当前登录用户信息
    """
    user = get_user_by_id(user_id)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="用户不存在"
        )
    
    return user.to_dict() 