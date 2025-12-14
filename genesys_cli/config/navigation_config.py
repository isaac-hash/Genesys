from enum import Enum
from typing import Optional
from pydantic import BaseModel, Field

class DriveType(str, Enum):
    DIFFERENTIAL = "differential"
    OMNI = "omni"
    ACKERMANN = "ackermann"
    SKID_STEER = "skid_steer"
    LEGGED = "legged"

class EnvironmentType(str, Enum):
    INDOOR = "indoor"
    OUTDOOR = "outdoor"
    MIXED = "mixed"

class GeometryConfig(BaseModel):
    length: float = Field(..., gt=0, description="Robot length in meters")
    width: float = Field(..., gt=0, description="Robot width in meters")
    height: float = Field(..., gt=0, description="Robot height in meters")
    wheel_base: float = Field(..., gt=0, description="Wheel base in meters")
    wheel_radius: float = Field(..., gt=0, description="Wheel radius in meters")

class SensorConfig(BaseModel):
    lidar_2d: bool = Field(True, description="Enable 2D Lidar")
    lidar_3d: bool = Field(False, description="Enable 3D Lidar")
    depth_camera: bool = Field(False, description="Enable Depth Camera")
    rgb_camera: bool = Field(False, description="Enable RGB Camera")
    imu: bool = Field(True, description="Enable IMU")

class OdometryConfig(BaseModel):
    fuse_imu: bool = Field(True, description="Fuse IMU data into odometry using robot_localization")

class NavParams(BaseModel):
    environment: EnvironmentType = Field(EnvironmentType.INDOOR, description="Operating environment")
    max_linear_velocity: float = Field(0.5, gt=0, description="Maximum linear velocity in m/s")
    max_angular_velocity: float = Field(1.0, gt=0, description="Maximum angular velocity in rad/s")

class FrameDefaults(BaseModel):
    base_link: str = Field("base_link", const=True)
    odom: str = Field("odom", const=True)
    map: str = Field("map", const=True)
    scan_topic: str = Field("/scan", const=True)
    imu_topic: str = Field("/imu/data", const=True)

class NavigationConfig(BaseModel):
    robot_name: str = Field(..., min_length=1, pattern=r"^[a-z][a-z0-9_]*$", description="Robot name in snake_case")
    drive_type: DriveType = Field(DriveType.DIFFERENTIAL, description="Type of robot drive train")
    simulation: bool = Field(True, description="Enable simulation support (Gazebo)")
    geometry: GeometryConfig
    sensors: SensorConfig = Field(default_factory=SensorConfig)
    odometry: OdometryConfig = Field(default_factory=OdometryConfig)
    params: NavParams = Field(default_factory=NavParams)
    frames: FrameDefaults = Field(default_factory=FrameDefaults)
