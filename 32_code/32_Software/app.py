"""
robotNav - 机器人导航系统
主入口文件

典型使用方式：
    from app import main
    
    if __name__ == '__main__':
        main()
"""

import sys
import logging
import time
from pathlib import Path

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

logger = logging.getLogger(__name__)


def setup_path():
    """配置Python路径"""
    root_dir = Path(__file__).parent
    sys.path.insert(0, str(root_dir))


def list_available_ports():
    """列出可用的串口"""
    from comm.serial_comm import list_available_ports
    
    ports = list_available_ports()
    if not ports:
        logger.warning("未找到可用的串口")
        return None
    
    logger.info(f"可用串口: {ports}")
    
    if len(ports) == 1:
        return ports[0]
    
    # 交互式选择
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port}")
    
    choice = input("请选择串口 (输入数字): ").strip()
    try:
        idx = int(choice) - 1
        if 0 <= idx < len(ports):
            return ports[idx]
    except:
        pass
    
    logger.error("无效的选择")
    return None


def interactive_demo():
    """交互式演示程序"""
    from robot_navigator import create_robot_navigator
    from core.settings import RobotPose
    
    logger.info("=== robotNav 导航系统演示 ===")
    logger.info("选择工作模式:")
    logger.info("  1. 自主探索")
    logger.info("  2. 目标导航")
    logger.info("  3. 手动测试")
    
    mode = input("请选择 (1-3): ").strip()
    
    # 获取串口
    com_port = list_available_ports()
    if not com_port:
        logger.error("无法获取串口")
        return
    
    logger.info(f"使用串口: {com_port}")
    
    # 创建导航器
    try:
        navigator = create_robot_navigator(com_port)
    except Exception as e:
        logger.error(f"创建导航器失败: {e}")
        return
    
    # 启动导航系统
    if not navigator.start():
        logger.error("启动导航系统失败")
        return
    
    logger.info("导航系统已启动")
    
    try:
        # 设置起始位姿
        start_pose = RobotPose(x_m=0.0, y_m=0.0, heading_deg=0.0)
        navigator.set_start_pose(start_pose)
        
        if mode == "1":
            # 探索模式
            navigator.set_exploration_mode()
            logger.info("进入探索模式，按 Ctrl+C 停止")
            
            while True:
                status = navigator.get_current_status()
                print(f"[{time.strftime('%H:%M:%S')}] "
                      f"位置: ({status.current_pose.x_m:.2f}, {status.current_pose.y_m:.2f}) | "
                      f"探索率: {status.map_exploration_ratio:.1f}% | "
                      f"路径长: {status.path_length_nodes}")
                time.sleep(2)
        
        elif mode == "2":
            # 目标导航模式
            goal_x = float(input("输入目标X坐标 (米): "))
            goal_y = float(input("输入目标Y坐标 (米): "))
            
            navigator.set_goal((goal_x, goal_y))
            logger.info(f"目标已设置: ({goal_x}, {goal_y})")
            logger.info("执行导航，按 Ctrl+C 停止")
            
            while True:
                status = navigator.get_current_status()
                print(f"[{time.strftime('%H:%M:%S')}] "
                      f"位置: ({status.current_pose.x_m:.2f}, {status.current_pose.y_m:.2f}) | "
                      f"任务模式: {status.task_mode.name}")
                time.sleep(2)
        
        else:
            logger.info("进入手动测试模式")
            logger.info("命令:")
            logger.info("  e - 探索模式")
            logger.info("  g - 设置目标")
            logger.info("  s - 显示状态")
            logger.info("  q - 退出")
            
            while True:
                cmd = input("> ").strip().lower()
                
                if cmd == "e":
                    navigator.set_exploration_mode()
                    logger.info("已切换到探索模式")
                
                elif cmd == "g":
                    goal_x = float(input("目标X (米): "))
                    goal_y = float(input("目标Y (米): "))
                    navigator.set_goal((goal_x, goal_y))
                    logger.info(f"目标已设置")
                
                elif cmd == "s":
                    status = navigator.get_current_status()
                    print(f"  位置: ({status.current_pose.x_m:.2f}, {status.current_pose.y_m:.2f})")
                    print(f"  航向: {status.current_pose.heading_deg:.1f}°")
                    print(f"  任务模式: {status.task_mode.name}")
                    print(f"  探索率: {status.map_exploration_ratio:.1f}%")
                
                elif cmd == "q":
                    break
    
    except KeyboardInterrupt:
        logger.info("用户中断")
    
    finally:
        navigator.stop()
        logger.info("导航系统已停止")


def main():
    """主入口"""
    setup_path()
    
    try:
        interactive_demo()
    except Exception as e:
        logger.error(f"程序异常: {e}", exc_info=True)
        return 1
    
    return 0


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
