"""
地图管理模块
集成BreezySLAM进行SLAM定位与建图，维护栅格地图
"""

import logging
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser
    BREEZY_AVAILABLE = True
except ImportError:
    BREEZY_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning("BreezySLAM未安装")

from core.settings import MapConfig, SLAMConfig, RobotPose


logger = logging.getLogger(__name__)


@dataclass
class MapStats:
    """地图统计信息"""
    free_cells: int         # 自由空间格子数
    obstacle_cells: int     # 障碍物格子数
    unknown_cells: int      # 未知区域格子数
    exploration_ratio: float  # 探索率（%）


class GridMapManager:
    """
    栅格地图管理器
    - 集成BreezySLAM进行位置估计
    - 维护栅格地图并逐步更新
    - 提供地图查询接口
    """
    
    def __init__(self, 
                 map_size_pix: int = MapConfig.SIZE_PIXELS,
                 map_size_m: float = MapConfig.SIZE_METERS):
        """
        初始化地图管理器
        
        Args:
            map_size_pix: 地图尺寸（像素）
            map_size_m: 地图尺寸（米）
        """
        self.map_size_pix = map_size_pix
        self.map_size_m = map_size_m
        self.pixel_scale = map_size_m / map_size_pix  # 米/像素
        
        # 主地图（栅格值：0-255）
        self.grid_map = np.full(
            (map_size_pix, map_size_pix), 
            MapConfig.UNKNOWN_VALUE, 
            dtype=np.uint8
        )
        
        # 投票机制用于地图融合
        self.obstacle_votes = np.zeros((map_size_pix, map_size_pix), dtype=np.int16)
        self.free_votes = np.zeros((map_size_pix, map_size_pix), dtype=np.int16)
        
        # 锁定地图（已确认的区域不再更新）
        self.locked_mask = np.zeros((map_size_pix, map_size_pix), dtype=bool)
        
        # BreezySLAM SLAM引擎
        self.slam_engine: Optional[RMHC_SLAM] = None
        self._init_slam()
        
        # 当前位姿
        self.current_pose = RobotPose(x_m=0.0, y_m=0.0, heading_deg=0.0)
        
        # 地图历史（用于增量更新检测）
        self.prev_slam_map = None
        self.changed_regions: List[Tuple[int, int, int, int]] = []
        
        logger.info(f"地图管理器已初始化: {map_size_pix}×{map_size_pix}像素, {map_size_m}×{map_size_m}米")
    
    def _init_slam(self):
        """初始化BreezySLAM引擎"""
        if not BREEZY_AVAILABLE:
            logger.error("BreezySLAM不可用，SLAM功能将被禁用")
            return
        
        try:
            # 创建激光雷达传感器对象
            # URG-04LX-UG01 参数
            lidar = Laser(
                scan_size=SLAMConfig.SCAN_SIZE,
                scan_rate_hz=SLAMConfig.SCAN_RATE_HZ,
                detection_angle_deg=SLAMConfig.DETECTION_ANGLE_DEG,
                distance_no_detection_mm=SLAMConfig.DISTANCE_NO_DETECTION_MM
            )
            
            # 创建SLAM算法实例
            self.slam_engine = RMHC_SLAM(
                laser=lidar,
                map_size_pixels=self.map_size_pix,
                map_size_meters=self.map_size_m,
                random_seed=SLAMConfig.RANDOM_SEED
            )
            
            logger.info("BreezySLAM引擎初始化成功")
        
        except Exception as e:
            logger.error(f"BreezySLAM初始化失败: {e}")
            self.slam_engine = None
    
    def update_from_scan(self, lidar_scan: List[int], 
                         lin_vel_mps: float, ang_vel_dps: float,
                         dt_sec: float) -> bool:
        """
        使用激光雷达扫描和里程计数据更新SLAM和地图
        
        Args:
            lidar_scan: 激光雷达扫描数组（毫米）
            lin_vel_mps: 线速度（米/秒）
            ang_vel_dps: 角速度（度/秒）
            dt_sec: 时间间隔（秒）
        
        Returns:
            是否更新成功
        """
        if not self.slam_engine:
            logger.warning("SLAM引擎不可用")
            return False
        
        try:
            # 将速度转换为mm/s和度/s（BreezySLAM使用的单位）
            lin_vel_mmps = lin_vel_mps * 1000
            dt_ms = int(dt_sec * 1000)
            
            # 更新SLAM
            self.slam_engine.update(lidar_scan, (lin_vel_mmps, ang_vel_dps, dt_ms))
            
            # 获取当前位姿（毫米、度）
            pos_x_mm, pos_y_mm, yaw_deg = self.slam_engine.getpos()
            
            # 转换为米
            self.current_pose.x_m = pos_x_mm / 1000.0
            self.current_pose.y_m = pos_y_mm / 1000.0
            self.current_pose.heading_deg = yaw_deg
            
            # 获取SLAM生成的地图并进行融合
            slam_map_buffer = bytearray(self.map_size_pix * self.map_size_pix)
            self.slam_engine.getmap(slam_map_buffer)
            slam_map = np.frombuffer(slam_map_buffer, dtype=np.uint8).reshape(
                self.map_size_pix, self.map_size_pix
            )
            
            # 更新地图（投票融合机制）
            self._fuse_map(slam_map)
            
            return True
        
        except Exception as e:
            logger.error(f"地图更新失败: {e}")
            return False
    
    def _fuse_map(self, slam_map: np.ndarray):
        """
        融合SLAM生成的地图到主地图（投票机制）
        
        Args:
            slam_map: SLAM生成的地图
        """
        # 检测变化区域
        if self.prev_slam_map is None:
            # 首次更新，全图处理
            self.changed_regions = [(0, 0, self.map_size_pix, self.map_size_pix)]
        else:
            # 增量检测
            change_mask = slam_map != self.prev_slam_map
            self.changed_regions = self._find_changed_regions(change_mask)
        
        # 投票更新
        for region in self.changed_regions:
            x0, y0, x1, y1 = region
            for y in range(y0, y1):
                for x in range(x0, x1):
                    if self.locked_mask[y, x]:
                        continue
                    
                    val = slam_map[y, x]
                    
                    # 根据值进行投票
                    if val <= MapConfig.OBSTACLE_THRESHOLD:
                        self.obstacle_votes[y, x] += 1
                    elif val >= MapConfig.FREE_THRESHOLD:
                        self.free_votes[y, x] += 1
        
        # 更新最终地图和锁定状态
        self._update_final_map()
        self.prev_slam_map = slam_map.copy()
    
    def _find_changed_regions(self, change_mask: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        找出地图变化的区域（矩形）
        
        Args:
            change_mask: 变化掩码
        
        Returns:
            变化区域列表 [(x0, y0, x1, y1), ...]
        """
        regions = []
        
        # 简化处理：扫描并合并相邻变化点
        rows, cols = np.where(change_mask)
        if len(rows) == 0:
            return []
        
        # 取变化区域的边界框
        x_min, x_max = cols.min(), cols.max()
        y_min, y_max = rows.min(), rows.max()
        
        # 膨胀边界以包含周围变化
        expand = 5
        x_min = max(0, x_min - expand)
        y_min = max(0, y_min - expand)
        x_max = min(self.map_size_pix - 1, x_max + expand)
        y_max = min(self.map_size_pix - 1, y_max + expand)
        
        regions.append((x_min, y_min, x_max + 1, y_max + 1))
        return regions
    
    def _update_final_map(self):
        """根据投票结果更新最终地图和锁定状态"""
        for y in range(self.map_size_pix):
            for x in range(self.map_size_pix):
                if self.locked_mask[y, x]:
                    continue
                
                obs_votes = self.obstacle_votes[y, x]
                free_votes = self.free_votes[y, x]
                total = obs_votes + free_votes
                
                # 低票数过滤
                if total < MapConfig.NOISE_FILTER_VOTES:
                    continue
                
                # 根据投票比例决定锁定
                if obs_votes >= MapConfig.OBSTACLE_LOCK_VOTES and \
                   obs_votes >= free_votes * MapConfig.CONFLICT_RATIO:
                    self.grid_map[y, x] = 0  # 障碍物
                    self.locked_mask[y, x] = True
                
                elif free_votes >= MapConfig.FREE_LOCK_VOTES and \
                     free_votes >= obs_votes * MapConfig.CONFLICT_RATIO:
                    self.grid_map[y, x] = 255  # 自由空间
                    self.locked_mask[y, x] = True
                
                else:
                    # 未达到确定阈值，更新为当前最有可能的值
                    if obs_votes > free_votes:
                        self.grid_map[y, x] = 50
                    elif free_votes > obs_votes:
                        self.grid_map[y, x] = 200
    
    def get_cell_value(self, x_pix: int, y_pix: int) -> int:
        """获取指定像素的栅格值"""
        if 0 <= x_pix < self.map_size_pix and 0 <= y_pix < self.map_size_pix:
            return int(self.grid_map[y_pix, x_pix])
        return MapConfig.UNKNOWN_VALUE
    
    def world_to_pixel(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """世界坐标转像素坐标"""
        x_pix = int((x_m / self.map_size_m + 0.5) * self.map_size_pix)
        y_pix = int((y_m / self.map_size_m + 0.5) * self.map_size_pix)
        return (x_pix, y_pix)
    
    def pixel_to_world(self, x_pix: int, y_pix: int) -> Tuple[float, float]:
        """像素坐标转世界坐标"""
        x_m = (x_pix / self.map_size_pix - 0.5) * self.map_size_m
        y_m = (y_pix / self.map_size_pix - 0.5) * self.map_size_m
        return (x_m, y_m)
    
    def is_passable(self, x_pix: int, y_pix: int) -> bool:
        """检查某点是否可通行（自由空间）"""
        val = self.get_cell_value(x_pix, y_pix)
        return val >= MapConfig.FREE_THRESHOLD
    
    def is_obstacle(self, x_pix: int, y_pix: int) -> bool:
        """检查某点是否是障碍物"""
        val = self.get_cell_value(x_pix, y_pix)
        return val <= MapConfig.OBSTACLE_THRESHOLD
    
    def get_stats(self) -> MapStats:
        """获取地图统计信息"""
        free_count = np.sum(self.grid_map >= MapConfig.FREE_THRESHOLD)
        obstacle_count = np.sum(self.grid_map <= MapConfig.OBSTACLE_THRESHOLD)
        unknown_count = self.map_size_pix ** 2 - free_count - obstacle_count
        
        exploration_ratio = (free_count + obstacle_count) / (self.map_size_pix ** 2) * 100
        
        return MapStats(
            free_cells=int(free_count),
            obstacle_cells=int(obstacle_count),
            unknown_cells=int(unknown_count),
            exploration_ratio=exploration_ratio
        )
    
    def save_map(self, filepath: str):
        """保存地图为图像文件"""
        try:
            from PIL import Image
            img = Image.fromarray(self.grid_map, mode='L')
            img.save(filepath)
            logger.info(f"地图已保存: {filepath}")
        except ImportError:
            logger.warning("PIL库不可用，无法保存地图图像")
        except Exception as e:
            logger.error(f"地图保存失败: {e}")
