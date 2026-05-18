# launch/main.py
# 전체 시스템 실행 진입점

import sys
import os

sys.path.insert(0,
    os.path.join(os.path.dirname(__file__), ".."))

from config.settings import ROS_DOMAIN_ID
os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from core.robot_controller import RobotController, DualRobotController
from vision.ceiling_camera import CeilingCamera
from core.robot_coordinator import RobotCoordinator


def run_single(robot_id: str = "pinky1"):
    """단일 로봇 실행 + 천장 카메라"""
    camera      = CeilingCamera()
    coordinator = RobotCoordinator(camera)
    coordinator.start()

    rclpy.init()
    robot    = RobotController(robot_id)
    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.stop()
        rclpy.shutdown()


def run_dual():
    """두 로봇 동시 실행 + 천장 카메라 조율"""
    camera      = CeilingCamera()
    coordinator = RobotCoordinator(camera)
    coordinator.start()

    ctrl = DualRobotController()
    try:
        ctrl.spin()
    finally:
        coordinator.stop()


# ── 실행 ───────────────────────────────────────────
# 단일 로봇: python3 launch/main.py single pinky1
# 두 로봇:   python3 launch/main.py dual
if __name__ == "__main__":
    mode     = sys.argv[1] if len(sys.argv) > 1 else "dual"
    robot_id = sys.argv[2] if len(sys.argv) > 2 else "pinky1"

    if mode == "single":
        run_single(robot_id)
    else:
        run_dual()
