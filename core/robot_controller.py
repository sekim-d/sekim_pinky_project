# core/robot_controller.py
# 메인 컨트롤러 — 모든 모듈 통합

import threading

import rclpy
import uvicorn
from fastapi import FastAPI
from geometry_msgs.msg import Twist
from pydantic import BaseModel
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from config.settings import ROBOT_CONFIG, ARUCO_CONFIG, WAYPOINTS
from navigation.nav_manager import NavManager
from navigation.slam_manager import SlamManager
from sensors.sensor_manager import SensorManager
from utils.logger import RobotLogger
from vision.aruco_detector import ArucoDetector
from vision.yolo_detector import YoloDetector
from core.visual_dock import VisualDock, MIN_DOCK_SIZE


class CommandRequest(BaseModel):
    action: str
    parameters: dict = {}


class RobotController(Node):
    """
    단일 로봇 통합 컨트롤러
    ─────────────────────────────────────────
    [로봇 제공] SensorManager  — 모든 센서 토픽 수신
    [로봇 제공] NavManager     — Nav2 액션 + 로봇 서비스
    [내가 추가] SlamManager    — SLAM 지도 생성/관리
    [내가 추가] ArucoDetector  — ArUco 마커 감지
    [내가 추가] YoloDetector   — 사람/박스 감지
    [웹서버 연동] FastAPI       — JSON 명령 수신
    """

    def __init__(self, robot_id: str = "pinky1"):
        super().__init__(f"controller_{robot_id}")

        cfg           = ROBOT_CONFIG[robot_id]
        self.robot_id = robot_id
        self.ns       = cfg["namespace"]
        self.log      = RobotLogger(self)

        self.log.info("SYS", f"=== {cfg['name']} 시작 ===")

        # ── 모듈 초기화 ────────────────────────────
        self.sensors       = SensorManager(self, self.ns)
        self.nav           = NavManager(self, self.ns)
        self.slam          = SlamManager(self)
        self.aruco         = ArucoDetector(self)
        self.yolo          = YoloDetector(self)
        self._docking           = False
        self._target_marker_id  = None   # 정밀 정차 대상 마커 ID
        self._searching         = False  # 마커 탐색 회전 중
        self._search_timer      = None
        self.visual_dock        = VisualDock(self)
        self._cmd_pub           = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── 콜백 연결 ──────────────────────────────
        self.sensors.on_image           = self._on_image
        self.sensors.on_person_detected = self._on_person_by_robot
        self.sensors.on_robot_status    = self._on_robot_status

        self.yolo.on_person_detected = self._on_person_by_yolo
        self.yolo.on_box_detected    = self._on_box_detected

        self.aruco.on_detected = self._on_aruco_detected

        # ── 비전 루프 타이머 (10Hz) ─────────────────
        self.create_timer(0.1, self._vision_loop)

        # ── HTTP API 서버 시작 ─────────────────────
        self._start_api_server(cfg["api_port"])

        self.log.info("SYS", "모든 모듈 초기화 완료")

    # ── API 서버 ───────────────────────────────────
    def _start_api_server(self, port: int):
        app = FastAPI()

        @app.post("/command")
        def receive_command(req: CommandRequest):
            self._run_command({"action": req.action,
                               "parameters": req.parameters})
            return {"status": "ok"}

        @app.post("/commands")
        def receive_commands(reqs: list[CommandRequest]):
            for req in reqs:
                self._run_command({"action": req.action,
                                   "parameters": req.parameters})
            return {"status": "ok"}

        @app.get("/status")
        def get_status():
            return {
                "robot_id":     self.robot_id,
                "position":     self.sensors.position,
                "battery":      self.sensors.battery_pct,
                "navigating":   self.nav.is_navigating,
                "map_ready":    self.slam.is_ready,
                "us_distance":  self.sensors.us_distance,
            }

        t = threading.Thread(
            target=uvicorn.run,
            args=(app,),
            kwargs={"host": "0.0.0.0", "port": port,
                    "log_level": "warning"},
            daemon=True)
        t.start()
        self.log.info("SYS",
            f"API 서버 시작 → http://0.0.0.0:{port}")

    # ── 비전 루프 ──────────────────────────────────
    def _vision_loop(self):
        img  = self.sensors.image
        cam  = self.sensors.camera_matrix
        dist = self.sensors.dist_coeffs

        if img is None:
            self._no_img_count = getattr(self, "_no_img_count", 0) + 1
            if self._no_img_count % 50 == 1:
                self.log.warn("SYS", "카메라 이미지 없음 (카메라 토픽 확인 필요)")
            return

        self._no_img_count = 0
        self.yolo.detect(img)
        self.aruco.detect(img, cam, dist)

    def _on_image(self, msg):
        pass

    # ── 이벤트 핸들러 ──────────────────────────────
    def _on_person_by_yolo(self, obj):
        self.log.warn("SYS", "YOLO 사람 감지 → 긴급 정지!")
        self.nav.emergency_stop(activate=True)
        self.nav.set_emotion("sad")
        self.nav.set_led(255, 0, 0)

    def _on_person_by_robot(self, msg):
        self.log.warn("SYS", "로봇 감지 사람 → 긴급 정지!")
        self.nav.emergency_stop(activate=True)

    def _on_box_detected(self, obj):
        self.log.info("SYS",
            f"박스 감지 위치: ({obj['cx']:.0f}, {obj['cy']:.0f})")

    def _on_aruco_detected(self, markers):
        for m in markers:

            if self._target_marker_id is not None and m["id"] == self._target_marker_id:
                if self.visual_dock.active:
                    self.visual_dock.update(m)
                elif not self._docking:
                    if m["pixel_size"] < MIN_DOCK_SIZE:
                        return  # 아직 멀어서 Nav2에 장애물 회피 맡김
                    self.log.info("SYS",
                        f"마커 {m['id']} 근접 감지 → 네비게이션 취소 후 정밀 정차 시작")
                    self.nav.cancel_navigation()
                    self._stop_search()
                    self._docking = True
                    self.visual_dock.start(done_callback=self._on_dock_done)
                    self.visual_dock.update(m)

    def _on_dock_done(self, result=None):
        self._docking          = False
        self._target_marker_id = None
        self.log.info("SYS", "정밀 정차 완료")

    def _on_nav_done(self, success=True):
        if not success or self._target_marker_id is None or self._docking:
            return
        self.log.info("SYS",
            f"도착 — 마커 {self._target_marker_id} 탐색 시작")
        self._searching = True
        if self._search_timer is None:
            self._search_timer = self.create_timer(0.1, self._search_spin)

    def _search_spin(self):
        if not self._searching:
            return
        if self._docking or self.visual_dock.active:
            self._stop_search()
            return
        msg = Twist()
        msg.angular.z = 0.3
        self._cmd_pub.publish(msg)

    def _stop_search(self):
        self._searching = False
        if self._search_timer is not None:
            self._search_timer.cancel()
            self._search_timer = None
        msg = Twist()
        self._cmd_pub.publish(msg)

    def _on_robot_status(self, msg):
        self.log.info("SYS", f"로봇 상태: {msg}")

    # ── 명령 실행 ──────────────────────────────────
    def _run_command(self, cmd: dict):
        action = cmd.get("action", "unknown")
        params = cmd.get("parameters", {})

        self.log.info("SYS", f"실행: {action} {params}")

        if action == "navigate":
            self._stop_search()
            self._target_marker_id = None
            location = params.get("location", "home")
            if location == "stop":
                self.nav.cancel_navigation()
                self.nav.emergency_stop(activate=True)
                return
            # 마커와 연결된 장소면 정밀 정차 대상 설정
            loc_to_marker = {v: k for k, v in ARUCO_CONFIG["marker_map"].items()}
            if location.startswith("marker_"):
                try:
                    self._target_marker_id = int(location.split("_")[1])
                except (IndexError, ValueError):
                    self._target_marker_id = None
            else:
                self._target_marker_id = loc_to_marker.get(location, None)
            if self._target_marker_id is not None:
                self.log.info("SYS",
                    f"목적지 {location} → 마커 {self._target_marker_id} 정밀 정차 대기")
            if location in WAYPOINTS:
                self.log.info("SYS",
                    f"웨이포인트 경로 이동 ({len(WAYPOINTS[location])}개)")
                self.nav.go_through(WAYPOINTS[location], callback=self._on_nav_done)
            else:
                self.nav.go_to_location(location, callback=self._on_nav_done)

        elif action == "dock":
            self.nav.dock_to_marker(
                int(params.get("marker_id", 0)))

        elif action == "transport":
            self.nav.run_transport(
                params.get("pickup",   "loading_zone"),
                params.get("delivery", "zone_A"))

        elif action == "stop":
            self._stop_search()
            self._target_marker_id = None
            self.nav.cancel_navigation()
            self.nav.emergency_stop(activate=True)
            self.nav.set_led(255, 0, 0)

        elif action == "resume":
            self.nav.emergency_stop(activate=False)
            self.nav.set_led(0, 255, 0)

        elif action == "start_mapping":
            self.slam.start_mapping()

        elif action == "start_localization":
            self.slam.start_localization(
                params.get("path", None))

        elif action == "stop_slam":
            self.slam.stop()

        elif action == "save_map":
            self.slam.save_map(params.get("path", None))

        elif action == "set_emotion":
            self.nav.set_emotion(
                params.get("emotion", "neutral"))

        elif action == "set_led":
            self.nav.set_led(
                int(params.get("r", 255)),
                int(params.get("g", 255)),
                int(params.get("b", 255)))

        elif action == "set_lamp":
            self.nav.set_lamp(
                params.get("mode",  "on"),
                params.get("color", "white"))

        elif action == "unknown":
            self.log.warn("SYS",
                f"모르는 명령: {cmd.get('message')}")


# ──────────────────────────────────────────────────
class DualRobotController:
    """
    두 로봇 동시 제어
    - pinky1 (port 8001) + pinky2 (port 8002)
    - MultiThreadedExecutor 로 병렬 처리
    """

    def __init__(self):
        rclpy.init()
        self.pinky1 = RobotController("pinky1")
        self.pinky2 = RobotController("pinky2")
        self.log    = RobotLogger(self.pinky1)
        self.log.info("SYS", "=== 듀얼 로봇 컨트롤러 준비 ===")

    def spin(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.pinky1)
        executor.add_node(self.pinky2)
        try:
            executor.spin()
        except KeyboardInterrupt:
            self.log.info("SYS", "종료 중...")
        finally:
            rclpy.shutdown()
