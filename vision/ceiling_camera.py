# vision/ceiling_camera.py
# 천장 USB 카메라로 ArUco 마커 감지 → 로봇 구역 추적

import cv2
import threading
from config.settings import CEILING_CAMERA_CONFIG


class CeilingCamera:
    def __init__(self):
        cfg = CEILING_CAMERA_CONFIG
        self.cap         = cv2.VideoCapture(cfg["camera_index"])
        self.marker_map  = cfg["marker_map"]
        self.zones       = cfg["zones"]

        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.robot_zones: dict[str, int] = {}

        self.on_zone_enter = None   # callback(robot_id, zone)
        self.on_zone_exit  = None   # callback(robot_id, zone)

        self._running = False
        self._thread  = None

    # ── 시작 / 종료 ────────────────────────────────
    def start(self):
        if not self.cap.isOpened():
            print("[CAM] 천장 카메라 연결 실패 — 카메라 없이 계속 실행")
            return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[CAM] 천장 카메라 시작")

    def stop(self):
        self._running = False
        if self.cap.isOpened():
            self.cap.release()
        print("[CAM] 천장 카메라 종료")

    # ── 구역 계산 ───────────────────────────────────
    def _get_zone(self, x: int, frame_width: int) -> int:
        zone_width = frame_width / self.zones
        return min(int(x / zone_width) + 1, self.zones)

    # ── 메인 루프 ───────────────────────────────────
    def _loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            h, w = frame.shape[:2]
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id not in self.marker_map:
                        continue

                    robot_id = self.marker_map[marker_id]
                    cx       = int(corners[i][0][:, 0].mean())
                    cy       = int(corners[i][0][:, 1].mean())
                    zone     = self._get_zone(cx, w)

                    prev_zone = self.robot_zones.get(robot_id)
                    if prev_zone != zone:
                        if prev_zone is not None and self.on_zone_exit:
                            self.on_zone_exit(robot_id, prev_zone)
                        self.robot_zones[robot_id] = zone
                        if self.on_zone_enter:
                            self.on_zone_enter(robot_id, zone)

                    self._draw_robot(frame, corners[i], robot_id, zone, cx, cy)

            self._draw_zones(frame, h, w)
            cv2.imshow("Ceiling Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self._running = False
                break

        cv2.destroyAllWindows()

    # ── 시각화 ─────────────────────────────────────
    def _draw_zones(self, frame, h: int, w: int):
        zone_w = w // self.zones
        for i in range(1, self.zones):
            x = zone_w * i
            cv2.line(frame, (x, 0), (x, h), (0, 255, 255), 2)

        for i in range(self.zones):
            label = f"Zone {i + 1}"
            cv2.putText(frame, label,
                        (zone_w * i + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    def _draw_robot(self, frame, corner, robot_id: str,
                    zone: int, cx: int, cy: int):
        color = (0, 255, 0) if robot_id == "pinky1" else (255, 100, 0)
        pts   = corner[0].astype(int)
        cv2.polylines(frame, [pts], True, color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
        cv2.putText(frame, f"{robot_id} (Zone {zone})",
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
