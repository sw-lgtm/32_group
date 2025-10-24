"""
蓝牙串口通信模块
处理与STM32的底层通信、帧解析、CRC校验等
"""

from __future__ import annotations
import struct
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Tuple
from queue import Queue, Full, Empty

import serial
import serial.tools.list_ports

from core.settings import CommConfig


logger = logging.getLogger(__name__)


# ==================== 工具函数 ====================

def crc16_modbus(data: bytes) -> int:
    """计算CRC16校验值（Modbus协议）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 1) != 0:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ==================== 数据结构 ====================

@dataclass
class LidarMeasurement:
    """激光雷达单点测量数据"""
    quality: int            # 信号质量
    angle_deg: float        # 角度（度）
    distance_mm: float      # 距离（毫米）


@dataclass
class SensorFrame:
    """传感器上行帧数据（STM32→PC）"""
    timestamp_us: int           # 时间戳（微秒）
    command_id: int             # 命令ID
    motion_state: int           # 运动状态 (0=停止, 1=转弯, 2=直行)
    encoder_left: int           # 左轮编码器值
    encoder_right: int          # 右轮编码器值
    yaw_angle_deg: float        # 当前航向角（度）
    measurements: List[LidarMeasurement]  # 激光雷达测量点


# ==================== 串口通信类 ====================

class SerialComm:
    """
    蓝牙串口通信管理器
    
    典型用法：
        comm = SerialComm('COM7')
        comm.start()
        comm.send_motion_cmd(cmd_id=1, turn_rad=0.2, distance_m=0.15)
        frame = comm.receive_frame(timeout=1.0)
        comm.stop()
    """
    
    def __init__(self, port: str, baud: int = CommConfig.BAUD_RATE):
        """
        初始化通信对象
        
        Args:
            port: 串口名称（如 'COM7'）
            baud: 波特率（默认921600）
        """
        self.port_name = port
        self.baud_rate = baud
        self.serial_port: Optional[serial.Serial] = None
        
        # 通信线程管理
        self.is_running = False
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        
        # 帧缓冲队列
        self._frame_queue: Queue[SensorFrame] = Queue(maxsize=30)
        
        # 连接状态
        self.is_connected = False
        self.last_reconnect_time = 0
        
        # 统计信息
        self.tx_count = 0
        self.rx_count = 0
        self.error_count = 0
    
    def start(self) -> bool:
        """启动通信线程"""
        if self.is_running:
            logger.warning("通信已启动")
            return False
        
        if not self._connect_serial():
            logger.error("无法连接串口")
            return False
        
        self.is_running = True
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        
        logger.info(f"通信已启动 - 端口: {self.port_name}, 波特率: {self.baud_rate}")
        return True
    
    def stop(self):
        """停止通信线程"""
        if not self.is_running:
            return
        
        self.is_running = False
        self._stop_event.set()
        
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
        
        logger.info("通信已停止")
    
    def send_motion_cmd(self, cmd_id: int, turn_rad: float, distance_m: float) -> bool:
        """
        发送运动控制命令
        
        帧格式：
            0xAA 0x55 | uint16 len=10 | uint16 cmd_id | 
            float32 turn_rad | float32 distance_m | uint16 CRC16
        
        Args:
            cmd_id: 命令ID
            turn_rad: 转角（弧度）
            distance_m: 距离（米）
        
        Returns:
            是否发送成功
        """
        if not self.is_connected:
            logger.warning("串口未连接，无法发送命令")
            return False
        
        try:
            # 构造数据段
            data = struct.pack('<HfF', cmd_id, turn_rad, distance_m)
            
            # 计算CRC
            crc = crc16_modbus(data)
            
            # 完整帧：帧头 + 长度 + 数据 + CRC
            frame = (CommConfig.HEADER_DOWNLINK + 
                    struct.pack('<H', len(data)) + 
                    data + 
                    struct.pack('<H', crc))
            
            self.serial_port.write(frame)
            self.tx_count += 1
            
            logger.debug(f"发送命令: ID={cmd_id}, 转角={turn_rad:.3f}rad, 距离={distance_m:.3f}m")
            return True
            
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
            self.error_count += 1
            return False
    
    def receive_frame(self, timeout: float = 1.0) -> Optional[SensorFrame]:
        """
        接收一帧数据（阻塞）
        
        Args:
            timeout: 等待超时时间（秒）
        
        Returns:
            SensorFrame或None
        """
        try:
            frame = self._frame_queue.get(timeout=timeout)
            return frame
        except Empty:
            return None
    
    def receive_latest_frame(self) -> Optional[SensorFrame]:
        """
        获取最新一帧数据（非阻塞），丢弃中间帧
        
        Returns:
            最新的SensorFrame或None
        """
        latest = None
        try:
            while True:
                latest = self._frame_queue.get_nowait()
        except Empty:
            pass
        
        return latest
    
    # ==================== 私有方法 ====================
    
    def _connect_serial(self) -> bool:
        """建立串口连接"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=CommConfig.TIMEOUT_SEC
            )
            self.is_connected = True
            logger.info(f"串口连接成功: {self.port_name}")
            return True
        except Exception as e:
            logger.error(f"串口连接失败: {e}")
            return False
    
    def _rx_loop(self):
        """接收线程主循环"""
        while self.is_running and not self._stop_event.is_set():
            if not self.is_connected:
                self._try_reconnect()
                continue
            
            try:
                frame = self._read_frame()
                if frame:
                    self.rx_count += 1
                    try:
                        self._frame_queue.put(frame, block=False)
                    except Full:
                        logger.warning("帧队列已满，丢弃旧帧")
                        try:
                            self._frame_queue.get_nowait()
                            self._frame_queue.put(frame, block=False)
                        except:
                            pass
            
            except Exception as e:
                logger.error(f"接收循环异常: {e}")
                self.error_count += 1
                self.is_connected = False
                time.sleep(0.1)
    
    def _read_frame(self) -> Optional[SensorFrame]:
        """
        从串口读取一帧数据
        
        帧格式：
            0x55 0xAA | uint32 time_us | uint16 cmd_id | uint8 status |
            int32 encoder_l | int32 encoder_r | float32 yaw |
            uint16 data_count | [data_count * {uint8 quality, uint16 angle_q6, uint16 dist_q2}]
        """
        if not self.serial_port or not self.serial_port.is_open:
            return None
        
        try:
            # 寻找帧头
            while True:
                byte1 = self.serial_port.read(1)
                if not byte1:
                    return None
                
                if byte1 == b'\x55':
                    byte2 = self.serial_port.read(1)
                    if byte2 == b'\xAA':
                        break
            
            # 读取帧头后的固定部分（22字节）
            header_data = self.serial_port.read(22)
            if len(header_data) < 22:
                return None
            
            # 解析固定部分
            (timestamp_us, cmd_id, status, 
             encoder_l, encoder_r, yaw_deg, data_count) = struct.unpack(
                '<IHBIIHI', header_data
            )
            
            # 读取激光雷达数据
            measurements = []
            for _ in range(data_count):
                point_data = self.serial_port.read(5)  # quality(1) + angle_q6(2) + dist_q2(2)
                if len(point_data) < 5:
                    break
                
                quality, angle_q6, dist_q2 = struct.unpack('<BHH', point_data)
                angle_deg = angle_q6 / 64.0
                distance_mm = dist_q2 / 4.0
                
                # 过滤无效测距
                if distance_mm < CommConfig.MAX_LIDAR_RANGE_MM:
                    measurements.append(LidarMeasurement(
                        quality=quality,
                        angle_deg=angle_deg,
                        distance_mm=distance_mm
                    ))
            
            return SensorFrame(
                timestamp_us=timestamp_us,
                command_id=cmd_id,
                motion_state=status,
                encoder_left=encoder_l,
                encoder_right=encoder_r,
                yaw_angle_deg=yaw_deg,
                measurements=measurements
            )
        
        except Exception as e:
            logger.error(f"帧解析失败: {e}")
            return None
    
    def _try_reconnect(self):
        """尝试重新连接"""
        current_time = time.time()
        if current_time - self.last_reconnect_time < CommConfig.RECONNECT_INTERVAL_SEC:
            time.sleep(0.5)
            return
        
        self.last_reconnect_time = current_time
        logger.info("尝试重新连接...")
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
        
        if self._connect_serial():
            logger.info("重新连接成功")
        else:
            logger.warning("重新连接失败")


# ==================== 工具函数 ====================

def list_available_ports() -> List[str]:
    """列出所有可用的串口"""
    ports = []
    for port_info in serial.tools.list_ports.comports():
        ports.append(port_info.device)
    return ports


def convert_lidar_to_scan(measurements: List[LidarMeasurement], 
                         scan_size: int = 682) -> List[int]:
    """
    将激光雷达测量点转换为扫描数组（用于SLAM）
    
    Args:
        measurements: 激光雷达测量点列表
        scan_size: 扫描点总数
    
    Returns:
        扫描数组（毫米），无效点为0
    """
    scan = [0] * scan_size
    
    for meas in measurements:
        # 将角度转换为索引
        angle_index = int((meas.angle_deg / 360.0) * scan_size) % scan_size
        
        # 质量检查
        if meas.quality >= CommConfig.MIN_QUALITY:
            scan[angle_index] = int(meas.distance_mm)
    
    return scan
