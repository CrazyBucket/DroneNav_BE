import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from datetime import datetime
import uvicorn
from routers.websocket import router as websocket_router
from starlette.middleware.cors import CORSMiddleware
from starlette.testclient import TestClient
from routers.http import router as http_router
from routers.auth import router as auth_router
import ssl

app = FastAPI(
    title="DroneNav API",
    description="无人机导航系统API",
    version="1.0.0",
)

app.include_router(websocket_router)
app.include_router(http_router)
app.include_router(auth_router)  # 添加认证路由

# 配置CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://localhost:5173"],  # 前端开发服务器地址
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 创建SSL上下文
def create_ssl_context():
    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(
        certfile="config/ssl/cert.pem",
        keyfile="config/ssl/key.pem"
    )
    return ssl_context

if __name__ == "__main__":
    # 创建SSL证书目录
    import os
    ssl_dir = "config/ssl"
    os.makedirs(ssl_dir, exist_ok=True)
    
    # 检查是否存在证书文件，如果不存在则生成自签名证书
    cert_path = f"{ssl_dir}/cert.pem"
    key_path = f"{ssl_dir}/key.pem"
    
    if not (os.path.exists(cert_path) and os.path.exists(key_path)):
        print("生成自签名SSL证书...")
        from cryptography import x509
        from cryptography.x509.oid import NameOID
        from cryptography.hazmat.primitives import hashes
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.hazmat.primitives import serialization
        import datetime
        
        # 生成私钥
        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=2048,
        )
        
        # 生成自签名证书
        subject = issuer = x509.Name([
            x509.NameAttribute(NameOID.COUNTRY_NAME, u"CN"),
            x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, u"Beijing"),
            x509.NameAttribute(NameOID.LOCALITY_NAME, u"Beijing"),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, u"DroneNav"),
            x509.NameAttribute(NameOID.COMMON_NAME, u"localhost"),
        ])
        
        cert = x509.CertificateBuilder().subject_name(
            subject
        ).issuer_name(
            issuer
        ).public_key(
            private_key.public_key()
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            datetime.datetime.utcnow()
        ).not_valid_after(
            datetime.datetime.utcnow() + datetime.timedelta(days=365)
        ).add_extension(
            x509.SubjectAlternativeName([x509.DNSName(u"localhost")]),
            critical=False,
        ).sign(private_key, hashes.SHA256())
        
        # 写入证书和私钥
        with open(key_path, "wb") as f:
            f.write(private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.PKCS8,
                encryption_algorithm=serialization.NoEncryption()
            ))
            
        with open(cert_path, "wb") as f:
            f.write(cert.public_bytes(serialization.Encoding.PEM))
        
        print(f"SSL证书已生成: {cert_path}, {key_path}")
    
    # 使用SSL启动服务器
    uvicorn.run(
        "main:app",  # 将应用作为导入字符串传递，而不是直接传递app对象
        host="0.0.0.0",
        port=8001,
        ssl_keyfile=key_path,
        ssl_certfile=cert_path,
        reload=True,
        ws='websockets',
        headers=[("Server", "DroneControl/1.0.0")]  # 添加自定义响应头
    )