from typing import Literal, Union
from pydantic import BaseModel, Field, confloat


class Feature(BaseModel):
    """公共特征基类，包含所有障碍物类型的共有属性"""
    pass


class ObstacleBase(BaseModel):
    """
    障碍物基础模型
    - id: 唯一标识符
    - type: 障碍物类型（CUBE/CYLINDER/TREE）
    - position: 三维坐标位置 (x, y, z)
    - rotation: 三维旋转角度 (x, y, z)，默认无旋转
    - metadata: 扩展元数据
    """
    id: str = Field(..., min_length=1, description="障碍物唯一标识符")
    type: str = Field(..., description="障碍物类型")
    position: tuple[confloat(ge=0), confloat(ge=0), confloat(ge=0)] = Field(
        ..., description="三维坐标位置 (x, y, z)")
    rotation: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description="三维旋转角度 (x, y, z)，单位：度")
    metadata: dict = Field(
        default_factory=dict,
        description="扩展元数据存储")


class CubeFeature(Feature):
    """立方体障碍物特征参数"""
    size: tuple[float, float, float] = Field(
        (5.0, 3.0, 10.0),
        description="尺寸参数 [宽度, 深度, 高度]")

    class Config:
        schema_extra = {
            "example": {
                "size": [5.0, 3.0, 10.0]
            }
        }


class CylinderFeature(Feature):
    """圆柱体障碍物特征参数"""
    radius: float = Field(2.0, gt=0, description="底面半径")
    height: float = Field(8.0, gt=0, description="圆柱高度")

    class Config:
        schema_extra = {
            "example": {
                "radius": 2.0,
                "height": 8.0
            }
        }


class TreeFeature(Feature):
    """树木障碍物特征参数"""
    trunk_radius: float = Field(0.3, gt=0, description="树干半径")
    canopy_size: float = Field(4.0, gt=0, description="树冠尺寸")

    class Config:
        schema_extra = {
            "example": {
                "trunkRadius": 0.3,
                "canopySize": 4.0
            }
        }


class CubeObstacle(ObstacleBase):
    """立方体障碍物模型"""
    type: Literal["CUBE"]
    feature: CubeFeature


class CylinderObstacle(ObstacleBase):
    """圆柱体障碍物模型"""
    type: Literal["CYLINDER"]
    feature: CylinderFeature


class TreeObstacle(ObstacleBase):
    """树木障碍物模型"""
    type: Literal["TREE"]
    feature: TreeFeature


Obstacle = Union[CubeObstacle, CylinderObstacle, TreeObstacle]