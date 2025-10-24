"""
robotNav - 机器人自主导航系统核心包
"""

__version__ = "1.0.0"
__author__ = "robotNav Team"
__license__ = "MIT"

# 核心模块导出
from core.settings import (
    TaskMode,
    RobotPose,
    ChassisParams,
    MovementLimits,
    NavigationTolerance,
    MotionCommand,
    MapConfig,
    ExplorationConfig,
    PlanningConfig,
    CommConfig,
    SLAMConfig,
    LogConfig
)

from robot_navigator import RobotNavigator, create_robot_navigator

__all__ = [
    'RobotNavigator',
    'create_robot_navigator',
    'TaskMode',
    'RobotPose',
    'ChassisParams',
    'MovementLimits',
    'NavigationTolerance',
    'MotionCommand',
    'MapConfig',
    'ExplorationConfig',
    'PlanningConfig',
    'CommConfig',
    'SLAMConfig',
    'LogConfig'
]
