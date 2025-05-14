from datetime import datetime
from typing import Optional, Dict, List
import uuid
import hashlib
import os
import base64
import hmac
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.backends import default_backend

# 内存中存储用户数据（实际项目中应使用数据库）
users_db: Dict[str, Dict] = {}

def generate_salt() -> str:
    """生成随机盐值"""
    return base64.b64encode(os.urandom(32)).decode('utf-8')

def hash_password(password: str, salt: str = None) -> tuple:
    """
    使用PBKDF2算法和SHA-256哈希函数对密码进行加密
    
    Args:
        password: 用户密码
        salt: 可选的盐值，如果不提供则生成新的
        
    Returns:
        (hashed_password, salt): 哈希后的密码和使用的盐值
    """
    if salt is None:
        salt = generate_salt()
    else:
        # 确保salt是字符串格式
        salt = salt if isinstance(salt, str) else salt.decode('utf-8')
    
    # 将密码和盐转换为字节
    password_bytes = password.encode('utf-8')
    salt_bytes = base64.b64decode(salt)
    
    # 使用PBKDF2HMAC进行密码哈希
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=salt_bytes,
        iterations=100000,
        backend=default_backend()
    )
    
    # 生成密钥
    key = kdf.derive(password_bytes)
    hashed_password = base64.b64encode(key).decode('utf-8')
    
    return hashed_password, salt

def verify_password(stored_password: str, stored_salt: str, provided_password: str) -> bool:
    """
    验证密码是否匹配
    
    Args:
        stored_password: 存储的哈希密码
        stored_salt: 存储的盐值
        provided_password: 用户提供的密码
        
    Returns:
        bool: 密码是否匹配
    """
    calculated_hash, _ = hash_password(provided_password, stored_salt)
    # 使用恒定时间比较避免计时攻击
    return hmac.compare_digest(calculated_hash, stored_password)

class User:
    """用户模型类"""
    def __init__(
        self,
        username: str,
        email: str,
        password: str,
        user_id: str = None,
        created_at: datetime = None,
        last_login: datetime = None,
        is_active: bool = True,
        role: str = "user"
    ):
        self.user_id = user_id if user_id else str(uuid.uuid4())
        self.username = username
        self.email = email
        self.created_at = created_at if created_at else datetime.now()
        self.last_login = last_login
        self.is_active = is_active
        self.role = role
        
        # 哈希密码
        self.password_hash, self.salt = hash_password(password)
    
    def verify_password(self, password: str) -> bool:
        """验证密码"""
        return verify_password(self.password_hash, self.salt, password)
    
    def update_last_login(self):
        """更新最后登录时间"""
        self.last_login = datetime.now()
    
    def to_dict(self, include_sensitive: bool = False) -> Dict:
        """转换为字典，可选是否包含敏感信息"""
        result = {
            "user_id": self.user_id,
            "username": self.username,
            "email": self.email,
            "created_at": self.created_at.isoformat(),
            "last_login": self.last_login.isoformat() if self.last_login else None,
            "is_active": self.is_active,
            "role": self.role
        }
        
        if include_sensitive:
            result.update({
                "password_hash": self.password_hash,
                "salt": self.salt
            })
            
        return result

def create_user(username: str, email: str, password: str) -> User:
    """创建新用户并存储"""
    # 检查用户名和邮箱是否已存在
    for user_data in users_db.values():
        if user_data["username"] == username:
            raise ValueError("用户名已存在")
        if user_data["email"] == email:
            raise ValueError("邮箱已存在")
    
    # 创建用户
    user = User(username=username, email=email, password=password)
    users_db[user.user_id] = user.to_dict(include_sensitive=True)
    
    return user

def get_user_by_id(user_id: str) -> Optional[User]:
    """通过ID获取用户"""
    if user_id not in users_db:
        return None
    
    user_data = users_db[user_id]
    user = User(
        user_id=user_data["user_id"],
        username=user_data["username"],
        email=user_data["email"],
        password="",  # 不传递实际密码
        created_at=datetime.fromisoformat(user_data["created_at"]),
        last_login=datetime.fromisoformat(user_data["last_login"]) if user_data["last_login"] else None,
        is_active=user_data["is_active"],
        role=user_data["role"]
    )
    # 手动设置密码哈希和盐
    user.password_hash = user_data["password_hash"]
    user.salt = user_data["salt"]
    
    return user

def get_user_by_username(username: str) -> Optional[User]:
    """通过用户名获取用户"""
    for user_id, user_data in users_db.items():
        if user_data["username"] == username:
            return get_user_by_id(user_id)
    return None

def get_user_by_email(email: str) -> Optional[User]:
    """通过邮箱获取用户"""
    for user_id, user_data in users_db.items():
        if user_data["email"] == email:
            return get_user_by_id(user_id)
    return None

# 初始化一些测试用户
def init_test_users():
    """初始化测试用户"""
    if not users_db:
        create_user("admin", "admin@dronenav.com", "Admin@123")
        create_user("test", "test@dronenav.com", "Test@123")
        print("已创建测试用户")

# 应用启动时初始化测试用户
init_test_users() 