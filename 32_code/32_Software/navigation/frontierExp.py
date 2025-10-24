"""
边界探索模块
检测地图前沿（已知和未知的边界）并选择最优探索目标
"""

import logging
import math
import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from enum import Enum, auto

from core.settings import ExplorationConfig, MapConfig


logger = logging.getLogger(__name__)


class TargetPriority(Enum):
    """探索目标优先级"""
    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


@dataclass
class BoundaryPoint:
    """边界探索点"""
    x_pix: int
    y_pix: int
    info_gain: float        # 信息增益评分
    distance_from_robot: float  # 与当前位置的距离
    priority: TargetPriority
    
    @property
    def composite_score(self) -> float:
        """综合评分（信息增益 + 距离）"""
        # 距离越近、信息增益越高 → 分数越高
        distance_penalty = 1.0 / (1.0 + self.distance_from_robot / 100.0)
        return self.info_gain * 0.7 + distance_penalty * 0.3


class BoundaryFinder:
    """
    前沿/边界探测器
    - 检测已知区域与未知区域的边界
    - 计算信息增益
    - 选择最优探索目标
    """
    
    def __init__(self,
                 safety_clearance: int = ExplorationConfig.SAFETY_CLEARANCE_PIX,
                 min_dist: int = ExplorationConfig.MIN_EXPLORATION_DIST_PIX,
                 max_dist: int = ExplorationConfig.MAX_EXPLORATION_DIST_PIX,
                 gain_radius: int = ExplorationConfig.GAIN_RADIUS_PIX):
        """
        初始化边界探测器
        
        Args:
            safety_clearance: 安全清间距离（像素）
            min_dist: 最小探索距离（像素）
            max_dist: 最大探索距离（像素）
            gain_radius: 信息增益计算半径（像素）
        """
        self.safety_clearance = safety_clearance
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.gain_radius = gain_radius
    
    def detect_boundaries(self, grid_map: np.ndarray) -> List[Tuple[int, int]]:
        """
        检测地图中所有的边界点
        
        边界点定义：自由空间但邻域内包含未知区域的格子
        
        Args:
            grid_map: 栅格地图
        
        Returns:
            边界点列表 [(x, y), ...]
        """
        height, width = grid_map.shape
        boundaries = []
        
        # 简单方法：直接扫描
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                val = grid_map[y, x]
                
                # 只考虑自由空间
                if val < MapConfig.FREE_THRESHOLD:
                    continue
                
                # 检查8邻域
                has_unknown = False
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        neighbor_val = grid_map[y + dy, x + dx]
                        if MapConfig.OBSTACLE_THRESHOLD < neighbor_val < MapConfig.FREE_THRESHOLD:
                            has_unknown = True
                            break
                    if has_unknown:
                        break
                
                if has_unknown:
                    boundaries.append((x, y))
        
        return boundaries
    
    def select_best_target(self, 
                          grid_map: np.ndarray,
                          robot_x_pix: int,
                          robot_y_pix: int,
                          num_candidates: int = 5) -> Optional[Tuple[int, int]]:
        """
        从边界点中选择最优探索目标
        
        Args:
            grid_map: 栅格地图
            robot_x_pix: 机器人X坐标（像素）
            robot_y_pix: 机器人Y坐标（像素）
            num_candidates: 返回的候选点数
        
        Returns:
            最优探索点 (x, y) 或 None
        """
        boundaries = self.detect_boundaries(grid_map)
        
        if not boundaries:
            logger.warning("未找到边界点")
            return None
        
        # 评估所有边界点
        candidates: List[BoundaryPoint] = []
        
        for x, y in boundaries:
            # 距离过滤
            dist = math.sqrt((x - robot_x_pix) ** 2 + (y - robot_y_pix) ** 2)
            
            if dist < self.min_dist or dist > self.max_dist:
                continue
            
            # 安全性检查（避开障碍物）
            if not self._is_safe_location(grid_map, x, y):
                continue
            
            # 计算信息增益
            info_gain = self._compute_information_gain(grid_map, x, y)
            
            # 确定优先级
            priority = self._assign_priority(info_gain, dist)
            
            candidates.append(BoundaryPoint(
                x_pix=x,
                y_pix=y,
                info_gain=info_gain,
                distance_from_robot=dist,
                priority=priority
            ))
        
        if not candidates:
            logger.warning("没有符合条件的探索候选点")
            return None
        
        # 按综合评分排序
        candidates.sort(key=lambda p: p.composite_score, reverse=True)
        
        # 返回最优点
        best = candidates[0]
        logger.debug(f"选择最优探索目标: ({best.x_pix}, {best.y_pix}), "
                    f"信息增益={best.info_gain:.2f}, 距离={best.distance_from_robot:.1f}pix")
        
        return (best.x_pix, best.y_pix)
    
    def select_multiple_targets(self,
                               grid_map: np.ndarray,
                               robot_x_pix: int,
                               robot_y_pix: int,
                               num_targets: int = 5) -> List[Tuple[int, int]]:
        """
        选择多个候选探索目标
        
        Args:
            grid_map: 栅格地图
            robot_x_pix: 机器人X坐标（像素）
            robot_y_pix: 机器人Y坐标（像素）
            num_targets: 返回的目标数量
        
        Returns:
            探索目标列表
        """
        boundaries = self.detect_boundaries(grid_map)
        
        if not boundaries:
            return []
        
        candidates: List[BoundaryPoint] = []
        
        for x, y in boundaries:
            dist = math.sqrt((x - robot_x_pix) ** 2 + (y - robot_y_pix) ** 2)
            
            if dist < self.min_dist or dist > self.max_dist:
                continue
            
            if not self._is_safe_location(grid_map, x, y):
                continue
            
            info_gain = self._compute_information_gain(grid_map, x, y)
            priority = self._assign_priority(info_gain, dist)
            
            candidates.append(BoundaryPoint(
                x_pix=x,
                y_pix=y,
                info_gain=info_gain,
                distance_from_robot=dist,
                priority=priority
            ))
        
        # 按优先级和评分排序
        candidates.sort(key=lambda p: (p.priority.value, p.composite_score), 
                       reverse=True)
        
        return [(c.x_pix, c.y_pix) for c in candidates[:num_targets]]
    
    # ==================== 私有方法 ====================
    
    def _is_safe_location(self, grid_map: np.ndarray, x: int, y: int) -> bool:
        """
        检查位置是否安全（周围没有障碍物）
        
        Args:
            grid_map: 栅格地图
            x, y: 位置坐标
        
        Returns:
            是否安全
        """
        height, width = grid_map.shape
        
        for dy in range(-self.safety_clearance, self.safety_clearance + 1):
            for dx in range(-self.safety_clearance, self.safety_clearance + 1):
                nx, ny = x + dx, y + dy
                
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                
                if grid_map[ny, nx] <= MapConfig.OBSTACLE_THRESHOLD:
                    return False
        
        return True
    
    def _compute_information_gain(self, grid_map: np.ndarray, x: int, y: int) -> float:
        """
        计算位置的信息增益（周围未知区域比例）
        
        Args:
            grid_map: 栅格地图
            x, y: 位置坐标
        
        Returns:
            信息增益值 (0-1)
        """
        height, width = grid_map.shape
        unknown_count = 0
        total_count = 0
        
        for dy in range(-self.gain_radius, self.gain_radius + 1):
            for dx in range(-self.gain_radius, self.gain_radius + 1):
                nx, ny = x + dx, y + dy
                
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                
                total_count += 1
                val = grid_map[ny, nx]
                
                if MapConfig.OBSTACLE_THRESHOLD < val < MapConfig.FREE_THRESHOLD:
                    unknown_count += 1
        
        if total_count == 0:
            return 0.0
        
        return unknown_count / total_count
    
    def _assign_priority(self, info_gain: float, distance: float) -> TargetPriority:
        """
        根据信息增益和距离分配优先级
        
        Args:
            info_gain: 信息增益
            distance: 距离
        
        Returns:
            优先级
        """
        # 高增益且距离近 → 高优先级
        if info_gain > 0.5 and distance < self.max_dist * 0.6:
            return TargetPriority.HIGH
        elif info_gain > 0.3:
            return TargetPriority.MEDIUM
        else:
            return TargetPriority.LOW
    
    def cluster_boundaries(self, 
                          boundaries: List[Tuple[int, int]],
                          cluster_distance: int = 50) -> List[Tuple[int, int]]:
        """
        将相近的边界点聚类成一个目标
        
        Args:
            boundaries: 边界点列表
            cluster_distance: 聚类距离阈值
        
        Returns:
            聚类后的中心点列表
        """
        if not boundaries:
            return []
        
        visited: Set[int] = set()
        clusters: List[List[Tuple[int, int]]] = []
        
        for i, (x1, y1) in enumerate(boundaries):
            if i in visited:
                continue
            
            cluster = [(x1, y1)]
            visited.add(i)
            
            for j, (x2, y2) in enumerate(boundaries):
                if j in visited:
                    continue
                
                dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if dist <= cluster_distance:
                    cluster.append((x2, y2))
                    visited.add(j)
            
            clusters.append(cluster)
        
        # 计算聚类中心
        centers = []
        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            centers.append((int(cx), int(cy)))
        
        return centers
