"""
robotNav - 核心配置文件
包含所有系统参数和常量定义
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Tuple, Optional


# ==================== 枚举类型 ====================

class TaskMode(Enum):
    """机器人任务模式"""
    EXPLORATION = auto()    # 自主探索模式
    HOMING = auto()         # 返回起点模式  
    GOAL_SEEKING = auto()   # 目标导航模式
    COMPLETED = auto()      # 任务完成


# ==================== 数据结构 ====================

@dataclass
class RobotPose:
    """机器人位姿（世界坐标系）"""
    x_m: float              # X坐标（米）
    y_m: float              # Y坐标（米）
    heading_deg: float      # 航向角（度，逆时针为正）


@dataclass
class ChassisParams:
    """底盘几何参数"""
    wheel_radius_m: float       # 车轮半径（米）
    wheelbase_half_m: float     # 半轮距（米）
    encoder_ppr: int            # 编码器每转脉冲数


@dataclass
class MovementLimits:
    """运动限制参数"""
    linear_speed_max: float = 0.25      # 最大线速度（米/秒）
    angular_speed_max: float = 90.0     # 最大角速度（度/秒）
    min_duration_sec: float = 0.2       # 最小运动时长（秒）
    max_duration_sec: float = 1.0       # 最大运动时长（秒）


@dataclass
class NavigationTolerance:
    """导航容差参数"""
    position_tol_m: float = 0.08        # 位置容差（米）
    heading_tol_deg: float = 35.0       # 航向容差（度）


@dataclass
class MotionCommand:
    """运动指令"""
    left_encoder_target: int            # 左轮目标编码器值
    right_encoder_target: int           # 右轮目标编码器值  
    duration_ms: int                    # 执行时长（毫秒）
    target_pose: RobotPose              # 目标位姿
    tolerance: NavigationTolerance      # 容差设置


# ==================== 地图参数 ====================

class MapConfig:
    """地图配置参数"""
    SIZE_PIXELS = 800                   # 地图尺寸（像素）
    SIZE_METERS = 6.0                   # 地图尺寸（米）
    
    # 灰度阈值
    FREE_THRESHOLD = 200                # 自由空间阈值
    OBSTACLE_THRESHOLD = 40             # 障碍物阈值
    UNKNOWN_VALUE = 128                 # 未知区域值
    
    # 投票机制参数
    OBSTACLE_LOCK_VOTES = 4             # 障碍物锁定票数
    FREE_LOCK_VOTES = 10                # 自由空间锁定票数
    CONFLICT_RATIO = 1                  # 冲突解决比率
    NOISE_FILTER_VOTES = 2              # 噪声过滤票数


# ==================== 探索参数 ====================

class ExplorationConfig:
    """探索配置参数"""
    # 边界点检测
    SAFETY_CLEARANCE_PIX = 35           # 安全距离（像素）
    MIN_EXPLORATION_DIST_PIX = 30       # 最小探索距离（像素）
    MAX_EXPLORATION_DIST_PIX = 250      # 最大探索距离（像素）
    
    # 信息增益计算
    GAIN_RADIUS_PIX = 8                 # 增益计算半径（像素）
    CONNECTIVITY_RADIUS_PIX = 200       # 连通性检查半径（像素）
    
    # 采样优化
    SPATIAL_SAMPLING_GRID = 20          # 空间采样网格大小
    MAX_CANDIDATES_EVAL = 20            # 最大候选点评估数


# ==================== 路径规划参数 ====================

class PlanningConfig:
    """路径规划配置参数"""
    ENABLE_DIAGONAL = True              # 允许对角移动
    INFLATION_RADIUS_PIX = 25           # 障碍物膨胀半径（像素）
    MAX_SNAP_RADIUS_PIX = 8             # 起终点吸附半径（像素）
    
    # 路径简化
    ENABLE_SIMPLIFICATION = True        # 启用路径简化
    TARGET_WAYPOINT_DIST_PIX = 250      # 目标路径点间距（像素）


# ==================== 串口通信参数 ====================

class CommConfig:
    """通信配置参数"""
    BAUD_RATE = 921600                  # 波特率
    TIMEOUT_SEC = 0.1                   # 读取超时（秒）
    RECONNECT_INTERVAL_SEC = 2.0        # 重连间隔（秒）
    
    # 帧头定义
    HEADER_DOWNLINK = b'\xAA\x55'       # 下行帧头（PC→STM32）
    HEADER_UPLINK = b'\x55\xAA'         # 上行帧头（STM32→PC）
    
    # 激光雷达参数
    MAX_LIDAR_RANGE_MM = 6000           # 最大有效测距（毫米）
    MIN_QUALITY = 0                     # 最低质量阈值


# ==================== BreezySLAM参数 ====================

class SLAMConfig:
    """SLAM配置参数"""
    RANDOM_SEED = 9999                  # 随机种子
    
    # URG-04LX-UG01 激光雷达参数
    SCAN_SIZE = 682                     # 扫描点数
    SCAN_RATE_HZ = 10.0                 # 扫描频率（Hz）
    DETECTION_ANGLE_DEG = 240.0         # 检测角度（度）
    DISTANCE_NO_DETECTION_MM = 4000     # 无检测时的默认距离（毫米）


# ==================== 日志配置 ====================

class LogConfig:
    """日志配置"""
    LEVEL = "INFO"                      # 日志级别
    FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    DATE_FORMAT = "%Y-%m-%d %H:%M:%S"
