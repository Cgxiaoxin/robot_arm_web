"""
统一运动命令类型与错误码定义。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


class MotionType(str, Enum):
    CARTESIAN_PTP = "CARTESIAN_PTP"
    CARTESIAN_LINEAR = "CARTESIAN_LINEAR"
    CARTESIAN_JOG = "CARTESIAN_JOG"


class Frame(str, Enum):
    BASE = "BASE"
    TOOL = "TOOL"
    USER = "USER"


class MotionState(str, Enum):
    ACCEPTED = "accepted"
    PLANNING = "planning"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
    PAUSED = "paused"


class ErrorCode(str, Enum):
    ARM_NOT_CONNECTED = "ARM_NOT_CONNECTED"
    IK_UNREACHABLE = "IK_UNREACHABLE"
    IK_LOW_ACCURACY = "IK_LOW_ACCURACY"
    SINGULARITY_NEAR = "SINGULARITY_NEAR"
    JOINT_LIMIT_REJECTED = "JOINT_LIMIT_REJECTED"
    CONSTRAINT_VIOLATION = "CONSTRAINT_VIOLATION"
    COMMAND_PREEMPTED = "COMMAND_PREEMPTED"
    CAN_TIMEOUT = "CAN_TIMEOUT"
    INTERNAL_ERROR = "INTERNAL_ERROR"
    COMMAND_IN_PROGRESS = "COMMAND_IN_PROGRESS"
    COMMAND_NOT_FOUND = "COMMAND_NOT_FOUND"


ERROR_MESSAGES = {
    ErrorCode.ARM_NOT_CONNECTED.value: "机械臂未连接，请先连接并初始化",
    ErrorCode.IK_UNREACHABLE.value: "目标位姿不可达，请减小位移或调整姿态",
    ErrorCode.IK_LOW_ACCURACY.value: "IK 精度不足，请降低速度并接近可操作区",
    ErrorCode.SINGULARITY_NEAR.value: "接近奇异位形，请调整姿态后再试",
    ErrorCode.JOINT_LIMIT_REJECTED.value: "关节超限，命令已拒绝",
    ErrorCode.CONSTRAINT_VIOLATION.value: "速度/加速度参数超范围",
    ErrorCode.COMMAND_PREEMPTED.value: "命令被新命令抢占",
    ErrorCode.CAN_TIMEOUT.value: "通讯超时，请检查总线与供电",
    ErrorCode.INTERNAL_ERROR.value: "内部错误，请查看系统日志",
    ErrorCode.COMMAND_IN_PROGRESS.value: "执行中禁止重复提交，请先取消/暂停或等待完成",
    ErrorCode.COMMAND_NOT_FOUND.value: "未找到指定命令",
}


@dataclass
class MotionCommand:
    command_id: str
    arm_id: str
    motion_type: MotionType
    frame: Frame = Frame.BASE
    target: Dict[str, Any] = field(default_factory=dict)
    constraints: Dict[str, float] = field(default_factory=dict)
    options: Dict[str, Any] = field(default_factory=dict)
    legacy_event: Optional[str] = None
    deprecated: bool = False

