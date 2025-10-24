"""
路径规划模块
使用A*或跳点搜索算法进行路径规划
"""

import logging
import heapq
import math
from typing import List, Tuple, Optional
import numpy as np

from core.settings import MapConfig, PlanningConfig


logger = logging.getLogger(__name__)


class RouteBuilder:
    """
    路径规划器
    - 使用A*或JPS算法规划从起点到终点的最优路径
    - 支持对角线移动
    - 支持在不可达时返回部分路径
    """
    
    def __init__(self,
                 inflation_radius: int = PlanningConfig.INFLATION_RADIUS_PIX,
                 allow_diagonal: bool = PlanningConfig.ENABLE_DIAGONAL,
                 free_threshold: int = MapConfig.FREE_THRESHOLD):
        """
        初始化路径规划器
        
        Args:
            inflation_radius: 障碍物膨胀半径（像素）
            allow_diagonal: 是否允许对角移动
            free_threshold: 自由空间阈值
        """
        self.inflation_radius = inflation_radius
        self.allow_diagonal = allow_diagonal
        self.free_threshold = free_threshold
    
    def plan_path(self, 
                  grid_map: np.ndarray,
                  start: Tuple[int, int],
                  goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        规划从起点到终点的路径
        
        Args:
            grid_map: 栅格地图
            start: 起点 (x, y)
            goal: 终点 (x, y)
        
        Returns:
            路径点列表或None
        """
        # 预处理地图（障碍物膨胀）
        inflated_map = self._inflate_obstacles(grid_map)
        
        # 调整起点和终点到可通行区域
        start = self._snap_to_free_space(inflated_map, start)
        goal = self._snap_to_free_space(inflated_map, goal)
        
        if start is None or goal is None:
            logger.error("起点或终点无法调整到可通行区域")
            return None
        
        # 执行A*搜索
        path = self._astar_search(inflated_map, start, goal)
        
        if path:
            # 简化路径
            path = self._simplify_path(grid_map, path)
            logger.info(f"路径规划成功: {len(path)}个节点")
            return path
        else:
            logger.warning("路径规划失败：无可行路径")
            return None
    
    def _inflate_obstacles(self, grid_map: np.ndarray) -> np.ndarray:
        """
        膨胀障碍物以增加安全间距
        
        Args:
            grid_map: 原始栅格地图
        
        Returns:
            膨胀后的地图
        """
        from scipy import ndimage
        
        # 创建障碍物掩码
        obstacle_mask = grid_map <= MapConfig.OBSTACLE_THRESHOLD
        
        # 膨胀
        if self.inflation_radius > 0:
            dilated = ndimage.binary_dilation(
                obstacle_mask,
                iterations=self.inflation_radius
            )
        else:
            dilated = obstacle_mask
        
        # 生成膨胀后的地图
        inflated = grid_map.copy()
        inflated[dilated] = 0  # 膨胀区域设为障碍物
        
        return inflated
    
    def _snap_to_free_space(self, 
                           grid_map: np.ndarray,
                           point: Tuple[int, int],
                           search_radius: int = 20) -> Optional[Tuple[int, int]]:
        """
        将点调整到最近的自由空间
        
        Args:
            grid_map: 栅格地图
            point: 待调整的点
            search_radius: 搜索半径
        
        Returns:
            调整后的点或None
        """
        x, y = point
        height, width = grid_map.shape
        
        # 检查原点是否已在自由空间
        if 0 <= x < width and 0 <= y < height:
            if grid_map[y, x] >= self.free_threshold:
                return (x, y)
        
        # 在搜索半径内寻找最近的自由空间
        for r in range(1, search_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dy) != r and abs(dx) != r:
                        continue
                    
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if grid_map[ny, nx] >= self.free_threshold:
                            return (nx, ny)
        
        return None
    
    def _astar_search(self,
                     grid_map: np.ndarray,
                     start: Tuple[int, int],
                     goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        执行A*搜索算法
        
        Args:
            grid_map: 栅格地图
            start: 起点
            goal: 终点
        
        Returns:
            路径或None
        """
        height, width = grid_map.shape
        
        def heuristic(p: Tuple[int, int]) -> float:
            """欧几里得距离启发"""
            dx = abs(p[0] - goal[0])
            dy = abs(p[1] - goal[1])
            return math.sqrt(dx * dx + dy * dy)
        
        def is_passable(x: int, y: int) -> bool:
            """检查格子是否可通行"""
            return (0 <= x < width and 0 <= y < height and
                   grid_map[y, x] >= self.free_threshold)
        
        # 开放集和关闭集
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start)}
        closed_set = set()
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            if current == goal:
                # 重构路径
                path = []
                node = goal
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                return path[::-1]
            
            closed_set.add(current)
            x, y = current
            
            # 扩展邻域
            neighbors = []
            
            # 4邻域（上下左右）
            for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                nx, ny = x + dx, y + dy
                if is_passable(nx, ny):
                    cost = 1.0
                    neighbors.append(((nx, ny), cost))
            
            # 8邻域（对角线）
            if self.allow_diagonal:
                for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if is_passable(nx, ny):
                        cost = 1.414  # sqrt(2)
                        neighbors.append(((nx, ny), cost))
            
            for neighbor, cost in neighbors:
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        return None
    
    def _simplify_path(self,
                      grid_map: np.ndarray,
                      path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        简化路径：移除不必要的中间点
        
        Args:
            grid_map: 栅格地图
            path: 原始路径
        
        Returns:
            简化后的路径
        """
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 尝试直接连接到更远的点
            farthest = i + 1
            
            for j in range(i + 2, len(path)):
                if self._is_line_of_sight(grid_map, path[i], path[j]):
                    farthest = j
            
            simplified.append(path[farthest])
            i = farthest
        
        return simplified
    
    def _is_line_of_sight(self,
                         grid_map: np.ndarray,
                         p1: Tuple[int, int],
                         p2: Tuple[int, int]) -> bool:
        """
        检查两点间是否有直线视线（Bresenham算法）
        
        Args:
            grid_map: 栅格地图
            p1: 点1
            p2: 点2
        
        Returns:
            是否有直线视线
        """
        x1, y1 = p1
        x2, y2 = p2
        
        # Bresenham直线算法
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x1 != x2:
                points.append((x1, y1))
                err -= dy
                if err < 0:
                    y1 += sy
                    err += dx
                x1 += sx
        else:
            err = dy / 2.0
            while y1 != y2:
                points.append((x1, y1))
                err -= dx
                if err < 0:
                    x1 += sx
                    err += dy
                y1 += sy
        
        points.append((x2, y2))
        
        # 检查路径上的所有点
        for x, y in points:
            if not (0 <= x < grid_map.shape[1] and 0 <= y < grid_map.shape[0]):
                return False
            if grid_map[y, x] < self.free_threshold:
                return False
        
        return True
