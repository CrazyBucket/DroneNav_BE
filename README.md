# DroneNav 后端服务

## 项目概述

DroneNav 是一个智能无人机导航系统的后端服务，提供无人机路径规划、障碍物避障、场景模拟等功能。该系统使用 FastAPI 构建 RESTful API 和 WebSocket 服务，支持实时数据传输和处理。

## 技术栈

- **Python 3.10+**
- **FastAPI**: 高性能的 Web 框架
- **Uvicorn**: ASGI 服务器
- **Pydantic**: 数据验证和设置管理
- **WebSockets**: 实时通信
- **NumPy**: 科学计算
- **Cryptography**: SSL/TLS 证书生成和管理

## 项目结构

```
DroneNav_BE/
├── config/         # 配置文件
│   └── ssl/        # SSL证书文件
├── core/           # 核心功能实现
├── models/         # 数据模型
├── obstacles/      # 障碍物处理
├── routers/        # API路由
├── scenarios/      # 预设场景
├── services/       # 业务服务层
├── tests/          # 单元测试
├── main.py         # 应用程序入口
└── requirements.txt # 依赖项
```

## 功能特性

- 无人机飞行路径规划
- 实时障碍物检测与避障
- 多种场景模拟
- WebSocket 实时数据传输
- RESTful API 接口
- HTTPS 和 WSS 安全通信

## 安装与运行

### 环境要求

- Python 3.10+

### 安装步骤

1. 克隆仓库

```bash
git clone <仓库地址>
cd DroneNav_BE
```

2. 创建虚拟环境（可选）

```bash
python -m venv venv
source venv/bin/activate  # 在Windows上使用: venv\Scripts\activate
```

3. 安装依赖

```bash
pip install -r requirements.txt
```

### 启动服务

```bash
python main.py
```

服务将在 https://0.0.0.0:8001 启动，自动生成自签名 SSL 证书

## API 文档

启动服务后，可以通过访问 https://localhost:8001/docs 查看 OpenAPI 文档。

## 测试

```bash
pytest
```

## 安全特性

### HTTPS 和 WSS

- 系统自动生成自签名 SSL 证书
- 所有 HTTP 通信通过 HTTPS 加密
- WebSocket 连接通过 WSS 加密
- 证书存储在 `config/ssl` 目录

### 注意事项

- 自签名证书在浏览器中可能会显示为不安全，这是正常的
- 在生产环境中，建议使用受信任的 CA 签发的证书
- 可以通过替换 `config/ssl` 目录中的证书文件来使用自己的证书

## 性能与可扩展性

- 使用异步处理支持高并发
- 模块化设计便于扩展新功能
- 单元测试覆盖率高

## 许可证

[添加许可证信息]
