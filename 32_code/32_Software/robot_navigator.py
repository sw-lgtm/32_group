"""
机器人导航主控制器
整合SLAM、地图、规划、通信等所有模块
"""

import logging
import threading
import time
from typing import Optional, List, Tuple
from dataclasses import dataclass

from core.settings import (
    TaskMode, RobotPose, ChassisParams, MovementLimits,
    NavigationTolerance, MapConfig, CommConfig
)
from comm.serial_comm import SerialComm, convert_lidar_to_scan
from mapping.grid_map_manager import GridMapManager
from planning.boundary_finder import BoundaryFinder
from planning.route_builder import RouteBuilder
from planning.motion_controller import MotionController


logger = logging.getLogger(__name__)


@dataclass
class NavigationStatus:
    """导航状态信息"""
    task_mode: TaskMode
    current_pose: RobotPose
    target_waypoint: Optional[RobotPose]
    map_exploration_ratio: float
    path_length_nodes: int
    notes: str


class RobotNavigator:
    """
    机器人导航主控制器
    
    责任：
    1. 管理通信（蓝牙、编码器、激光）
    2. 维护地图（SLAM + 栅格融合）
    3. 执行探索或目标导航任务
    4. 产生控制指令
    """
    
    def __init__(self,
                 com_port: str,
                 chassis_params: ChassisParams,
                 map_size_pix: int = MapConfig.SIZE_PIXELS,
                 map_size_m: float = MapConfig.SIZE_METERS):
        """
        初始化导航器
        
        Args:
            com_port: 通信串口（如 'COM7'）
            chassis_params: 底盘参数
            map_size_pix: 地图尺寸（像素）
            map_size_m: 地图尺寸（米）
        """
        self.com_port = com_port
        self.chassis = chassis_params
        
        # 通信模块
        self.comm = SerialComm(com_port, CommConfig.BAUD_RATE)
        
        # 地图模块
        self.map_manager = GridMapManager(map_size_pix, map_size_m)
        
        # 规划模块
        self.boundary_finder = BoundaryFinder()
        self.route_builder = RouteBuilder()
        self.motion_controller = MotionController(chassis_params)
        
        # 任务状态
        self.task_mode = TaskMode.EXPLORATION
        self.start_pose: Optional[RobotPose] = None
        self.goal_pose: Optional[RobotPose] = None
        self.current_waypoint: Optional[RobotPose] = None
        self.planned_path: List[Tuple[int, int]] = []
        
        # 探索状态
        self.frontier_candidates: List[Tuple[int, int]] = []
        self.best_frontier: Optional[Tuple[int, int]] = None
        
        # 控制循环
        self.is_running = False
        self._stop_event = threading.Event()
        self._control_thread: Optional[threading.Thread] = None
        
        # 统计信息
        self.frame_count = 0
        self.error_count = 0
        
        logger.info(f"机器人导航器已初始化: 端口={com_port}, 地图={map_size_pix}×{map_size_pix}")
    
    def start(self) -> bool:
        """启动导航系统"""
        if self.is_running:
            logger.warning("导航系统已启动")
            return False
        
        # 启动通信
        if not self.comm.start():
            logger.error("无法启动通信")
            return False
        
        # 启动控制循环
        self.is_running = True
        self._stop_event.clear()
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
        logger.info("导航系统已启动")
        return True
    
    def stop(self):
        """停止导航系统"""
        if not self.is_running:
            return
        
        self.is_running = False
        self._stop_event.set()
        
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        
        self.comm.stop()
        logger.info("导航系统已停止")
    
    def set_exploration_mode(self):
        """设置为探索模式"""
        self.task_mode = TaskMode.EXPLORATION
        logger.info("切换为探索模式")
    
    def set_goal(self, goal_world: Tuple[float, float]):
        """
        设置目标点（世界坐标）
        
        Args:
            goal_world: (x_m, y_m)
        """
        self.goal_pose = RobotPose(
            x_m=goal_world[0],
            y_m=goal_world[1],
            heading_deg=0.0
        )
        self.task_mode = TaskMode.GOAL_SEEKING
        logger.info(f"目标已设置: {goal_world}")
    
    def set_start_pose(self, pose: RobotPose):
        """设置起始位姿"""
        self.start_pose = pose
        self.map_manager.current_pose = pose
        self.motion_controller.update_pose(pose)
        logger.info(f"起始位姿已设置: ({pose.x_m:.2f}, {pose.y_m:.2f}, {pose.heading_deg:.1f}°)")
    
    def get_current_status(self) -> NavigationStatus:
        """获取当前导航状态"""
        stats = self.map_manager.get_stats()
        
        return NavigationStatus(
            task_mode=self.task_mode,
            current_pose=self.map_manager.current_pose,
            target_waypoint=self.current_waypoint,
            map_exploration_ratio=stats.exploration_ratio,
            path_length_nodes=len(self.planned_path),
            notes=""
        )
    
    # ==================== 私有方法 ====================
    
    def _control_loop(self):
        """主控制循环"""
        while self.is_running and not self._stop_event.is_set():
            try:
                # 接收传感器数据
                frame = self.comm.receive_frame(timeout=0.5)
                if not frame:
                    continue
                
                self.frame_count += 1
                
                # 更新地图和位姿（使用SLAM）
                scan_array = convert_lidar_to_scan(frame.measurements)
                lin_vel = 0.0  # 简化：从编码器计算
                ang_vel = float(frame.yaw_angle_deg)  # 简化
                dt = 0.1  # 帧间隔时间
                
                if not self.map_manager.update_from_scan(scan_array, lin_vel, ang_vel, dt):
                    logger.warning("地图更新失败")
                    self.error_count += 1
                
                # 更新里程计
                self.motion_controller.update_odometry(
                    frame.encoder_left,
                    frame.encoder_right,
                    frame.yaw_angle_deg,
                    time.time()
                )
                
                # 根据任务模式执行相应逻辑
                if self.task_mode == TaskMode.EXPLORATION:
                    self._explore()
                elif self.task_mode == TaskMode.GOAL_SEEKING:
                    self._seek_goal()
                
                # 控制周期
                time.sleep(0.01)
            
            except Exception as e:
                logger.error(f"控制循环异常: {e}")
                self.error_count += 1
                time.sleep(0.1)
    
    def _explore(self):
        """探索逻辑"""
        # 获取当前位置
        curr_pose = self.map_manager.current_pose
        robot_x_pix, robot_y_pix = self.map_manager.world_to_pixel(
            curr_pose.x_m, curr_pose.y_m
        )
        
        # 选择探索目标
        target_pix = self.boundary_finder.select_best_target(
            self.map_manager.grid_map,
            robot_x_pix,
            robot_y_pix
        )
        
        if not target_pix:
            logger.debug("未找到新的探索目标")
            return
        
        self.best_frontier = target_pix
        
        # 规划路径
        path_pix = self.route_builder.plan_path(
            self.map_manager.grid_map,
            (robot_x_pix, robot_y_pix),
            target_pix
        )
        
        if not path_pix:
            logger.warning("无法规划到目标的路径")
            return
        
        self.planned_path = path_pix
        
        # 转换为世界坐标生成路径点
        waypoints = []
        for x_pix, y_pix in path_pix[::10]:  # 每10个点取一个
            x_m, y_m = self.map_manager.pixel_to_world(x_pix, y_pix)
            waypoints.append(RobotPose(x_m=x_m, y_m=y_m, heading_deg=0.0))
        
        # 生成运动指令
        if waypoints:
            self.current_waypoint = waypoints[0]
            cmd = self.motion_controller.generate_motion_command(
                waypoints[0],
                duration_sec=0.5
            )
            
            if cmd:
                # 下发命令
                turn_rad = 0.0  # 简化
                distance_m = 0.1  # 简化
                self.comm.send_motion_cmd(
                    cmd_id=self.frame_count,
                    turn_rad=turn_rad,
                    distance_m=distance_m
                )
    
    def _seek_goal(self):
        """目标寻求逻辑"""
        if not self.goal_pose:
            return
        
        # 获取当前位置
        curr_pose = self.map_manager.current_pose
        robot_x_pix, robot_y_pix = self.map_manager.world_to_pixel(
            curr_pose.x_m, curr_pose.y_m
        )
        
        # 目标转换为像素
        goal_x_pix, goal_y_pix = self.map_manager.world_to_pixel(
            self.goal_pose.x_m, self.goal_pose.y_m
        )
        
        # 检查是否到达
        if self.motion_controller.is_target_reached(self.goal_pose):
            logger.info("已到达目标")
            self.task_mode = TaskMode.COMPLETED
            return
        
        # 规划路径
        path_pix = self.route_builder.plan_path(
            self.map_manager.grid_map,
            (robot_x_pix, robot_y_pix),
            (goal_x_pix, goal_y_pix)
        )
        
        if path_pix:
            self.planned_path = path_pix
            
            # 执行第一段路径
            if len(path_pix) > 1:
                next_pix = path_pix[1]
                next_x_m, next_y_m = self.map_manager.pixel_to_world(
                    next_pix[0], next_pix[1]
                )
                
                target = RobotPose(
                    x_m=next_x_m,
                    y_m=next_y_m,
                    heading_deg=self.goal_pose.heading_deg
                )
                
                cmd = self.motion_controller.generate_motion_command(target, 0.5)
                if cmd:
                    self.comm.send_motion_cmd(
                        cmd_id=self.frame_count,
                        turn_rad=0.0,
                        distance_m=0.1
                    )


# ==================== 工厂函数 ====================

def create_robot_navigator(com_port: str) -> RobotNavigator:
    """
    创建默认配置的机器人导航器
    
    Args:
        com_port: 通信串口
    
    Returns:
        RobotNavigator实例
    """
    # 标准底盘参数
    chassis = ChassisParams(
        wheel_radius_m=0.03,           # 30mm轮子
        wheelbase_half_m=0.075,        # 150mm轴距
        encoder_ppr=2000                # 2000线编码器
    )
    
    navigator = RobotNavigator(com_port, chassis)
    return navigator
