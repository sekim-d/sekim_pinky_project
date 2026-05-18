# config/settings.py
# 모든 설정값을 한 곳에서 관리
# 수정할 때 이 파일만 수정하면 됨

# ── 로봇 설정 ──────────────────────────────────────
ROBOT_CONFIG = {
    "pinky1": {
        "ip":        "192.168.4.6",
        "namespace": "",
        "name":      "핑키1 (적재 담당)",
        "api_port":  8001,
    },
    "pinky2": {
        "ip":        "192.168.4.6",
        "namespace": "",
        "name":      "핑키2 (배송 담당)",
        "api_port":  8002,
    },
}

ROS_DOMAIN_ID = 99

# ── 장소 좌표 (지도 기준) ──────────────────────────
LOCATIONS = {
    "home":           {"x": -0.8341, "y": -1.2818, "yaw": 0.4965},
    "loading_zone":   {"x": 1.0,  "y": 0.5,  "yaw": 0.0},
    "unloading_zone": {"x": 3.0,  "y": 0.5,  "yaw": 3.14},
    "zone_A":         {"x": -1.0361, "y": -0.9091, "yaw": 0.5682},
    "marker_0":       {"x": 0.0877,  "y": -0.6687, "yaw": 0.5356},
    "marker_1":       {"x": -0.6788, "y": -1.1592, "yaw": -1.0846},
    "zone_B":         {"x": 4.0,  "y": 2.0,  "yaw": 1.57},
    "zone_C":         {"x": 4.0,  "y": 3.0,  "yaw": 1.57},
    "charging":       {"x": 0.5,  "y": 0.5,  "yaw": 0.0},
    "warehouse":      {"x": 2.0,  "y": 1.5,  "yaw": 0.0},
}

# ── ArUco 설정 ─────────────────────────────────────
ARUCO_CONFIG = {
    "dict":              "DICT_4X4_250",
    "marker_size":       0.1,    # 실제 마커 크기 (m)
    "approach_distance": 0.3,    # 마커 앞 정차 거리 (m)
    "marker_map": {              # 마커ID → 장소 매핑
        0: "loading_zone",
        1: "unloading_zone",
        2: "zone_A",
        3: "zone_B",
        4: "zone_C",
    },
}

# ── YOLO 설정 ──────────────────────────────────────
YOLO_CONFIG = {
    "model":        "yolov8n.pt",
    "confidence":   0.6,
    "enabled":      False,       # 커스텀 모델 준비 후 True로 변경
    "classes":      [],
}

# ── Nav2 설정 ──────────────────────────────────────
NAV2_CONFIG = {
    "goal_tolerance": 0.1,       # 목적지 허용 오차 (m)
    "nav_timeout":    60.0,      # 이동 타임아웃 (초)
    "dock_timeout":   30.0,      # 도킹 타임아웃 (초)
}

# ── SLAM 설정 ──────────────────────────────────────
SLAM_CONFIG = {
    "map_frame":     "map",
    "odom_frame":    "odom",
    "base_frame":    "base_footprint",
    "map_save_path": "./maps/my_map",
}

# ── 천장 카메라 설정 ───────────────────────────────
CEILING_CAMERA_CONFIG = {
    "camera_index": 1,          # USB 카메라 번호 (0, 1, 2 ...)
    "zones":        3,          # 구역 수
    "critical_zone": 2,         # 충돌 위험 구역 (가운데)
    "marker_map": {             # 마커ID → 로봇 매핑
        10: "pinky1",
        11: "pinky2",
    },
}

# ── 로봇 토픽/서비스/액션 정의 ─────────────────────
# 로봇이 제공하는 인터페이스 목록
ROBOT_INTERFACES = {
    "topics": {
        "sub": {
            "cmd_vel":        "geometry_msgs/Twist",
        },
        "pub": {
            "odom":           "nav_msgs/Odometry",
            "scan":           "sensor_msgs/LaserScan",
            "camera_image":   "sensor_msgs/Image",
            "camera_info":    "sensor_msgs/CameraInfo",
            "imu":            "sensor_msgs/Imu",
            "battery":        "sensor_msgs/BatteryState",
            "us_sensor":      "sensor_msgs/Range",
            "ir_sensor":      "std_msgs/UInt16MultiArray",
            "robot_status":   "pinky_interfaces/RobotStatus",
            "mission_status": "pinky_interfaces/MissionStatus",
            "person_detected":"pinky_interfaces/PersonDetected",
        },
    },
    "services": {
        "set_emotion":    "pinky_interfaces/Emotion",
        "set_led":        "pinky_interfaces/SetLed",
        "set_brightness": "pinky_interfaces/SetBrightness",
        "set_lamp":       "pinky_interfaces/SetLamp",
        "emergency_stop": "pinky_interfaces/EmergencyStop",
    },
    "actions": {
        "navigate_to_pose":       "nav2_msgs/NavigateToPose",
        "navigate_through_poses": "nav2_msgs/NavigateThroughPoses",
        "transport_mission":      "pinky_interfaces/TransportMission",
        "dock_to_marker":         "pinky_interfaces/DockToMarker",
    },
}
