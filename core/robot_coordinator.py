# core/robot_coordinator.py
# 천장 카메라 기반 두 로봇 충돌 방지 조율

import threading
import requests
from config.settings import ROBOT_CONFIG, CEILING_CAMERA_CONFIG


class RobotCoordinator:
    """
    CeilingCamera 의 구역 진입/이탈 이벤트를 받아
    한 로봇이 중간 구역(critical_zone)을 통과하는 동안
    다른 로봇을 자동으로 대기/재개 시킴.
    """

    def __init__(self, camera):
        self.camera        = camera
        self.critical_zone = CEILING_CAMERA_CONFIG["critical_zone"]
        self._lock         = threading.Lock()
        self._occupant     = None   # critical_zone 에 현재 있는 로봇

        camera.on_zone_enter = self._on_zone_enter
        camera.on_zone_exit  = self._on_zone_exit

    # ── 시작 / 종료 ────────────────────────────────
    def start(self):
        self.camera.start()
        print(f"[COORD] 조율 시작 — 충돌 위험 구역: {self.critical_zone}번")

    def stop(self):
        self.camera.stop()

    # ── 내부 헬퍼 ──────────────────────────────────
    def _other_robot(self, robot_id: str) -> str | None:
        all_robots = list(self.camera.marker_map.values())
        return next((r for r in all_robots if r != robot_id), None)

    def _send(self, robot_id: str, action: str):
        cfg = ROBOT_CONFIG[robot_id]
        url = f"http://{cfg['ip']}:{cfg['api_port']}/command"
        try:
            requests.post(url,
                          json={"action": action, "parameters": {}},
                          timeout=2)
            print(f"[COORD] {robot_id} → {action}")
        except Exception as e:
            print(f"[COORD] {robot_id} 명령 실패: {e}")

    # ── 이벤트 핸들러 ──────────────────────────────
    def _on_zone_enter(self, robot_id: str, zone: int):
        if zone != self.critical_zone:
            return
        with self._lock:
            other = self._other_robot(robot_id)
            self._occupant = robot_id
            print(f"[COORD] {robot_id} → 구역{zone} 진입 | {other} 대기")
            if other:
                self._send(other, "stop")

    def _on_zone_exit(self, robot_id: str, zone: int):
        if zone != self.critical_zone:
            return
        with self._lock:
            if self._occupant != robot_id:
                return
            other = self._other_robot(robot_id)
            self._occupant = None
            print(f"[COORD] {robot_id} → 구역{zone} 통과 완료 | {other} 재개")
            if other:
                self._send(other, "resume")
