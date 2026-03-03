"""
Web控制系统配置
"""
import os
from pathlib import Path

# 项目根目录
PROJECT_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = Path(__file__).resolve().parents[1]

# CAN配置
DEFAULT_LEFT_CAN = "can0"
DEFAULT_RIGHT_CAN = "can1"
CAN_BITRATE = 1000000

# 电机配置
LEFT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]
RIGHT_MOTOR_IDS = [61, 62, 63, 64, 65, 66, 67]
ALL_MOTOR_IDS = LEFT_MOTOR_IDS + RIGHT_MOTOR_IDS

# 服务配置
HOST = "0.0.0.0"
PORT = 5000
DEBUG = os.getenv("DEBUG", "false").lower() == "true"

# 状态推送间隔 (毫秒)
STATE_UPDATE_INTERVAL = 200

# 轨迹文件目录
TRAJECTORIES_DIR = WEB_ROOT / "trajectories"

# 日志目录
LOG_DIR = WEB_ROOT / "logs"

# 零点偏移文件（与 arm_control 共用，连接时自动读入，标定时更新）
ZERO_OFFSET_FILE = WEB_ROOT / "zero_offset.json"

# 预设动作配置
PRESET_ACTIONS = {
    "A": "preset_a.json",
    "B": "preset_b.json",
    "C": "preset_c.json",
    "D": "preset_d.json",
    "E": "preset_e.json",
    "F": "preset_f.json",
}

################################################################################
# 速度与关节限制配置
################################################################################

# 速度预设
SPEED_PRESETS = {
    "very_slow": {"velocity": 0.2, "accel": 0.2, "decel": 0.2},
    "slow": {"velocity": 0.5, "accel": 0.5, "decel": 0.5},
    "medium": {"velocity": 1.0, "accel": 1.0, "decel": 1.0},
    "fast": {"velocity": 2.0, "accel": 2.0, "decel": 2.0},
}

# 软关节限位（弧度）
# 说明：
# - 默认采用较为保守的范围，避免在前端误操作导致大幅度摆动
# - 如果后续有更精确的驱动/机构限制，可在此处调整
JOINT_LIMITS = {
    "left": {
        # 对应 LEFT_MOTOR_IDS 顺序：51~57
        "position_min": [-2.5, -2.5, -2.0, -2.0, -2.5, -2.5, -2.5],
        "position_max": [2.5, 2.5, 2.0, 2.0, 2.5, 2.5, 2.5],
    },
    "right": {
        # 对应 RIGHT_MOTOR_IDS 顺序：61~67
        "position_min": [-2.5, -2.5, -2.0, -2.0, -2.5, -2.5, -2.5],
        "position_max": [2.5, 2.5, 2.0, 2.0, 2.5, 2.5, 2.5],
    },
}

