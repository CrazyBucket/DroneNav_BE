#!/bin/bash

# 显示彩色文本的函数
print_green() {
    echo -e "\033[0;32m$1\033[0m"
}

print_yellow() {
    echo -e "\033[0;33m$1\033[0m"
}

print_red() {
    echo -e "\033[0;31m$1\033[0m"
}

# 确保证书目录存在
mkdir -p config/ssl

# 检查是否存在证书文件
if [ ! -f "config/ssl/cert.pem" ] || [ ! -f "config/ssl/key.pem" ]; then
    print_yellow "证书文件不存在，将使用main.py生成..."
    
    # 直接使用python生成证书
    python -c "
import os
import ssl
from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import serialization
import datetime

ssl_dir = 'config/ssl'
cert_path = f'{ssl_dir}/cert.pem'
key_path = f'{ssl_dir}/key.pem'

# 生成私钥
private_key = rsa.generate_private_key(
    public_exponent=65537,
    key_size=2048,
)

# 生成自签名证书
subject = issuer = x509.Name([
    x509.NameAttribute(NameOID.COUNTRY_NAME, u'CN'),
    x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, u'Beijing'),
    x509.NameAttribute(NameOID.LOCALITY_NAME, u'Beijing'),
    x509.NameAttribute(NameOID.ORGANIZATION_NAME, u'DroneNav'),
    x509.NameAttribute(NameOID.COMMON_NAME, u'localhost'),
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
    x509.SubjectAlternativeName([x509.DNSName(u'localhost')]),
    critical=False,
).sign(private_key, hashes.SHA256())

# 写入证书和私钥
with open(key_path, 'wb') as f:
    f.write(private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption()
    ))
    
with open(cert_path, 'wb') as f:
    f.write(cert.public_bytes(serialization.Encoding.PEM))

print(f'SSL证书已生成: {cert_path}, {key_path}')
"
fi

# 检查证书是否已生成
if [ ! -f "config/ssl/cert.pem" ] || [ ! -f "config/ssl/key.pem" ]; then
    print_red "证书生成失败，无法启动HTTPS服务！"
    exit 1
fi

# 启动HTTPS服务
print_green "=============================="
print_green "    DroneNav HTTPS 服务      "
print_green "=============================="
print_green "使用自签名SSL证书启动HTTPS服务..."
print_green "服务地址: https://localhost:8001"
print_green "按CTRL+C退出"
print_green "=============================="

# 使用明确的参数启动uvicorn
uvicorn main:app \
    --host 0.0.0.0 \
    --port 8001 \
    --ssl-keyfile=config/ssl/key.pem \
    --ssl-certfile=config/ssl/cert.pem \
    --log-level info 