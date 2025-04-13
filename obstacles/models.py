from typing import Literal, Union, Optional, List
from pydantic import BaseModel, Field, field_validator, confloat

# ----------------- 基础类型 -----------------
class Position3D(BaseModel):
    x: float
    y: float
    z: float

class Rotation3D(BaseModel):
    pitch: float = Field(0.0, ge=0, le=360, description="绕X轴旋转角度（度）")
    yaw: float = Field(0.0, ge=0, le=360, description="绕Y轴旋转角度（度）")
    roll: float = Field(0.0, ge=0, le=360, description="绕Z轴旋转角度（度）")

# ----------------- 特征基类 -----------------
class Feature(BaseModel):
    """所有特征的基类"""
    pass

# ----------------- 障碍物类型 -----------------
class ObstacleBase(BaseModel):
    id: str = Field(..., min_length=1, description="唯一标识符")
    type: str = Field(..., description="障碍物类型")
    position: Position3D
    rotation: Rotation3D = Field(default_factory=Rotation3D)
    metadata: dict = Field(default_factory=dict)

# ----------------- 立方体 -----------------
class CubeFeature(Feature):
    size: tuple[float, float, float] = Field(
        ..., 
        description="[宽, 高, 深] 单位：米"
    )
    texture: Optional[str] = Field(
        None,
        description="可选贴图标识（如 'concrete', 'brick'）"
    )
    color: Optional[str] = Field(
        None,
        pattern="^#([A-Fa-f0-9]{6}|[A-Fa-f0-9]{3})$",
        description="十六进制颜色值（如 #FF0000）"
    )

    @field_validator('texture', 'color')
    
    @field_validator('texture', 'color')
    def check_material(cls, v, values):
        """验证器：确保贴图和颜色不同时存在"""
        if v and values.data.get('texture') and values.data.get('color'):
            raise ValueError("不能同时指定贴图和颜色")
        return v
    
class CubeObstacle(ObstacleBase):
    type: Literal["CUBE"]
    feature: CubeFeature

# ----------------- 圆柱体 -----------------
class CylinderFeature(Feature):
    radius: float = Field(..., gt=0, description="底面半径（米）")
    height: float = Field(..., gt=0, description="高度（米）")
    capped: bool = Field(True, description="是否包含顶面和底面")

class CylinderObstacle(ObstacleBase):
    type: Literal["CYLINDER"]
    feature: CylinderFeature

# ----------------- 树木 -----------------
class TreeStyle(BaseModel):
    model: str = Field(..., description="模型文件名（不含扩展名）")
    scale: float = Field(1.0, gt=0, description="缩放比例")

class TreeObstacle(ObstacleBase):
    type: Literal["TREE"]
    feature: TreeStyle

# ----------------- 建筑物 -----------------
class BuildingModelStyle(BaseModel):
    model_id: str = Field(..., description="预定义模型标识")

class BuildingTextureStyle(BaseModel):
    main_texture: str = Field(..., description="主墙面贴图标识")

class BuildingFeature(Feature):
    footprint: tuple[float, float] = Field(..., description="底面尺寸 [长, 宽] 米")
    height: float = Field(..., gt=0, description="建筑高度（米）")
    style: Union[BuildingModelStyle, BuildingTextureStyle] = Field(
        ...,
        description="模型风格配置"
    )

class BuildingObstacle(ObstacleBase):
    type: Literal["BUILDING"]
    feature: BuildingFeature

# ----------------- 道路系统 -----------------
class RoadMarking(BaseModel):
    type: Literal["center_line", "side_line", "turn_arrow"]
    pattern: Literal["solid", "dashed", "double"]
    color: str = Field("#ffffff", pattern="^#([A-Fa-f0-9]{6}|[A-Fa-f0-9]{3})$")

class RoadMaterial(BaseModel):
    surface: str = Field(..., description="路面材质标识")
    markings: List[RoadMarking] = Field(default_factory=list)

class StraightRoadFeature(Feature):
    length: float = Field(..., gt=0, description="道路长度（米）")
    direction: float = Field(0.0, ge=0, lt=360, description="方向角（度）")

class CurvedRoadFeature(Feature):
    radius: float = Field(..., gt=0, description="转弯半径（米）")
    angle: float = Field(..., gt=0, le=180, description="转弯角度（度）")
    direction: Literal["left", "right"]

class RoadSegment(ObstacleBase):
    width: float = Field(3.75, gt=0, description="道路宽度（米）")
    material: RoadMaterial
    feature: Union[StraightRoadFeature, CurvedRoadFeature]

    @property
    def type(self) -> Literal["ROAD"]:
        return "ROAD"

# ----------------- 联合类型 -----------------
Obstacle = Union[
    CubeObstacle,
    CylinderObstacle,
    TreeObstacle,
    BuildingObstacle,
    RoadSegment
]

class SceneConfig(BaseModel):
    name: str = Field(..., description="场景名称")
    coordinate_system: Literal["ENU", "NED"] = Field(
        "ENU",
        description="坐标系类型"
    )
    obstacles: List[Obstacle] = Field(
        default_factory=list,
        description="障碍物列表"
    )