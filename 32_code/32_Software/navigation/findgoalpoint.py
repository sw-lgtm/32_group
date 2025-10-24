"""
运动控制模块
处理位姿转换、轨迹规划、编码器命令生成
"""

import logging
import math
from typing import Tuple, List, Optional
from dataclasses import dataclass

from core.settings import (
    RobotPose, ChassisParams, MovementLimits,
    NavigationTolerance, MotionCommand
)


logger = logging.getLogger(__name__)


@dataclass
class OdometryData:
    """里程计数据"""
    timestamp_sec: float        # 时间戳
    left_ticks: int             # 左轮累计编码器值
    right_ticks: int            # 右轮累计编码器值
    yaw_angle_deg: float        # 航向角


class MotionController:
    """
    运动控制器
    - 管理机器人位姿、速度
    - 根据目标生成编码器控制指令
    - 处理里程计反馈
    """
    
    def __init__(self,
                 chassis: ChassisParams,
                 limits: MovementLimits = MovementLimits(),
                 tolerance: NavigationTolerance = NavigationTolerance()):
        """
        初始化运动控制器
        
        Args:
            chassis: 底盘参数
            limits: 运动限制
            tolerance: 导航容差
        """
        self.chassis = chassis
        self.limits = limits
        self.tolerance = tolerance
        
        # 当前位姿
        self.pose = RobotPose(x_m=0.0, y_m=0.0, heading_deg=0.0)
        
        # 里程计
        self.odometry = OdometryData(
            timestamp_sec=0.0,
            left_ticks=0,
            right_ticks=0,
            yaw_angle_deg=0.0
        )
        
        # 运动历史
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
    
    def update_pose(self, pose: RobotPose):
        """更新当前位姿"""
        self.pose = pose
    
    def update_odometry(self, left_ticks: int, right_ticks: int, 
                       yaw_angle_deg: float, timestamp_sec: float):
        """
        更新里程计数据
        
        Args:
            left_ticks: 左轮累计编码器值
            right_ticks: 右轮累计编码器值
            yaw_angle_deg: 航向角
            timestamp_sec: 时间戳
        """
        self.odometry.left_ticks = left_ticks
        self.odometry.right_ticks = right_ticks
        self.odometry.yaw_angle_deg = yaw_angle_deg
        self.odometry.timestamp_sec = timestamp_sec
    
    def calculate_motion_from_odometry(self) -> Tuple[float, float, float]:
        """
        从里程计差分计算本段运动
        
        Returns:
            (linear_velocity_mps, angular_velocity_dps, time_interval_sec)
        """
        # 计算编码器增量
        delta_left = self.odometry.left_ticks - self.prev_left_ticks
        delta_right = self.odometry.right_ticks - self.prev_right_ticks
        
        # 更新历史值
        self.prev_left_ticks = self.odometry.left_ticks
        self.prev_right_ticks = self.odometry.right_ticks
        
        # 转换为弧长（米）
        wheel_perimeter = 2 * math.pi * self.chassis.wheel_radius_m
        left_distance = (delta_left / self.chassis.encoder_ppr) * wheel_perimeter
        right_distance = (delta_right / self.chassis.encoder_ppr) * wheel_perimeter
        
        # 计算线速度和角速度
        linear_vel = (left_distance + right_distance) / 2.0
        
        # 轮距为2*half_wheelbase
        wheelbase = 2 * self.chassis.wheelbase_half_m
        angular_displacement = (right_distance - left_distance) / wheelbase  # 弧度
        angular_vel_dps = math.degrees(angular_displacement)
        
        return (linear_vel, angular_vel_dps, 0.0)  # 时间间隔在外部提供
    
    def generate_motion_command(self,
                               target_pose: RobotPose,
                               duration_sec: float) -> Optional[MotionCommand]:
        """
        生成运动指令：根据目标位姿和持续时间计算编码器目标值
        
        Args:
            target_pose: 目标位姿
            duration_sec: 指令执行时长
        
        Returns:
            MotionCommand或None
        """
        # 验证参数
        if duration_sec < self.limits.min_duration_sec:
            logger.warning(f"时长过短: {duration_sec}s < {self.limits.min_duration_sec}s")
            return None
        
        if duration_sec > self.limits.max_segment_time_s:
            logger.warning(f"时长过长: {duration_sec}s > {self.limits.max_segment_time_s}s")
            duration_sec = self.limits.max_segment_time_s
        
        # 计算相对位移
        dx = target_pose.x_m - self.pose.x_m
        dy = target_pose.y_m - self.pose.y_m
        distance_m = math.sqrt(dx * dx + dy * dy)
        
        # 计算相对航向
        delta_heading = target_pose.heading_deg - self.pose.heading_deg
        
        # 规范化角度到 [-180, 180]
        while delta_heading > 180:
            delta_heading -= 360
        while delta_heading < -180:
            delta_heading += 360
        
        # 根据持续时间计算所需速度
        lin_vel_mps = min(distance_m / duration_sec, self.limits.linear_speed_max)
        ang_vel_dps = min(abs(delta_heading) / duration_sec, 
                         self.limits.angular_speed_max) * (1 if delta_heading > 0 else -1)
        
        # 转换为编码器值
        wheel_perimeter = 2 * math.pi * self.chassis.wheel_radius_m
        dist_per_tick = wheel_perimeter / self.chassis.encoder_ppr
        
        # 根据线速度和角速度计算左右轮速度
        wheelbase = 2 * self.chassis.wheelbase_half_m
        ang_vel_rad = math.radians(ang_vel_dps)
        
        # 差速公式：v_left = v_lin - w * B
        #          v_right = v_lin + w * B
        v_left = lin_vel_mps - ang_vel_rad * self.chassis.wheelbase_half_m
        v_right = lin_vel_mps + ang_vel_rad * self.chassis.wheelbase_half_m
        
        # 计算编码器目标值
        left_distance = v_left * duration_sec
        right_distance = v_right * duration_sec
        
        left_ticks_target = int(left_distance / dist_per_tick)
        right_ticks_target = int(right_distance / dist_per_tick)
        
        # 转换为累计值
        left_ticks_cumulative = self.odometry.left_ticks + left_ticks_target
        right_ticks_cumulative = self.odometry.right_ticks + right_ticks_target
        
        cmd = MotionCommand(
            left_encoder_target=left_ticks_cumulative,
            right_encoder_target=right_ticks_cumulative,
            duration_ms=int(duration_sec * 1000),
            target_pose=target_pose,
            tolerance=self.tolerance
        )
        
        return cmd
    
    def is_target_reached(self, target_pose: RobotPose) -> bool:
        """
        检查是否到达目标位姿
        
        Args:
            target_pose: 目标位姿
        
        Returns:
            是否到达
        """
        # 位置距离
        dx = target_pose.x_m - self.pose.x_m
        dy = target_pose.y_m - self.pose.y_m
        distance = math.sqrt(dx * dx + dy * dy)
        
        # 航向差
        delta_heading = target_pose.heading_deg - self.pose.heading_deg
        while delta_heading > 180:
            delta_heading -= 360
        while delta_heading < -180:
            delta_heading += 360
        
        # 检查容差
        pos_ok = distance <= self.tolerance.position_tol_m
        heading_ok = abs(delta_heading) <= self.tolerance.heading_tol_deg
        
        return pos_ok and heading_ok
    
    def decompose_path_to_commands(self,
                                   waypoints: List[RobotPose]) -> List[MotionCommand]:
        """
        将路径分解为一系列运动指令
        
        Args:
            waypoints: 路径点列表
        
        Returns:
            运动指令列表
        """
        commands = []
        
        for waypoint in waypoints:
            # 计算应该耗时多久
            dx = waypoint.x_m - self.pose.x_m
            dy = waypoint.y_m - self.pose.y_m
            distance = math.sqrt(dx * dx + dy * dy)
            
            if distance < 0.01:  # 太近，跳过
                continue
            
            # 根据距离和最大速度计算时间
            duration = distance / self.limits.linear_speed_max
            duration = max(duration, self.limits.min_duration_sec)
            duration = min(duration, self.limits.max_segment_time_s)
            
            cmd = self.generate_motion_command(waypoint, duration)
            if cmd:
                commands.append(cmd)
                self.pose = waypoint  # 更新当前位姿估计
        
        return commands
    
    def regulate_speed(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """
        限制速度在允许范围内
        
        Args:
            linear_vel: 线速度（米/秒）
            angular_vel: 角速度（度/秒）
        
        Returns:
            调节后的 (linear_vel, angular_vel)
        """
        # 限制线速度
        if abs(linear_vel) > self.limits.linear_speed_max:
            linear_vel = self.limits.linear_speed_max * (1 if linear_vel > 0 else -1)
        
        # 限制角速度
        if abs(angular_vel) > self.limits.angular_speed_max:
            angular_vel = self.limits.angular_speed_max * (1 if angular_vel > 0 else -1)
        
        return (linear_vel, angular_vel)


class TrajectoryPlanner:
    """
    轨迹规划器
    - 根据起点、终点和约束规划平滑轨迹
    """
    
    def __init__(self, controller: MotionController):
        """
        初始化轨迹规划器
        
        Args:
            controller: 运动控制器
        """
        self.controller = controller
    
    def generate_arc_waypoints(self,
                              start: RobotPose,
                              goal: RobotPose,
                              num_waypoints: int = 10) -> List[RobotPose]:
        """
        生成圆弧插值的路径点
        
        Args:
            start: 起点
            goal: 终点
            num_waypoints: 路径点数
        
        Returns:
            插值后的路径点
        """
        waypoints = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # 线性插值位置
            x = start.x_m + (goal.x_m - start.x_m) * t
            y = start.y_m + (goal.y_m - start.y_m) * t
            
            # 航向角线性插值
            heading = start.heading_deg + (goal.heading_deg - start.heading_deg) * t
            
            waypoints.append(RobotPose(x_m=x, y_m=y, heading_deg=heading))
        
        return waypoints
