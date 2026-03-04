"""
群控管理器 - 管理多个机械臂的同步控制

支持:
- 左臂(can0) + 右臂(can1) 独立或同步控制
- 严格时序同步播放轨迹 (±10ms)
- 紧急停止
"""
import sys
import json
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from enum import Enum

# 添加项目根目录到路径，复用现有控制器
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from arm_control import ArmController, MODE_PROFILE_POSITION

from .config import (
    DEFAULT_LEFT_CAN,
    DEFAULT_RIGHT_CAN,
    TRAJECTORIES_DIR,
    SPEED_PRESETS,
    LEFT_MOTOR_IDS,
    RIGHT_MOTOR_IDS,
    JOINT_LIMITS,
    ZERO_OFFSET_FILE,
)


class ControlTarget(Enum):
    """控制目标"""
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


class PlaybackState(Enum):
    """播放状态"""
    IDLE = "idle"
    PLAYING = "playing"
    PAUSED = "paused"
    STOPPING = "stopping"


@dataclass
class ArmState:
    """单臂状态"""
    connected: bool = False
    initialized: bool = False
    can_channel: str = ""
    motors: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    error: Optional[str] = None


@dataclass
class PlaybackInfo:
    """播放信息"""
    state: PlaybackState = PlaybackState.IDLE
    trajectory_name: str = ""
    current_point: int = 0
    total_points: int = 0
    progress: float = 0.0


class GroupController:
    """
    群控管理器
    
    管理左右两个机械臂，支持独立控制或同步控制
    """
    
    def __init__(self):
        self.arms: Dict[str, Optional[ArmController]] = {
            "left": None,
            "right": None
        }
        self.arm_states: Dict[str, ArmState] = {
            "left": ArmState(can_channel=DEFAULT_LEFT_CAN),
            "right": ArmState(can_channel=DEFAULT_RIGHT_CAN)
        }
        
        self.target = ControlTarget.BOTH
        self.playback = PlaybackInfo()
        
        self._lock = threading.RLock()
        self._playback_thread: Optional[threading.Thread] = None
        self._stop_playback = threading.Event()
        self._pause_playback = threading.Event()
        
        # 当前速度参数（用于前端显示/回显）
        default_speed = SPEED_PRESETS.get("slow", {"velocity": 0.5, "accel": 0.5, "decel": 0.5})
        self.speed_params = {
            "velocity": float(default_speed.get("velocity", 1.0)),
            "accel": float(default_speed.get("accel", 1.0)),
            "decel": float(default_speed.get("decel", 1.0)),
        }
        
        # 回调函数
        self._on_state_update: Optional[Callable] = None
        self._on_playback_progress: Optional[Callable] = None
        self._on_error: Optional[Callable] = None

        # 预计算的软关节限位（按电机ID展开）
        self.joint_limits: Dict[str, Dict[int, Dict[str, float]]] = {
            "left": {},
            "right": {},
        }
        self._init_joint_limits()
    
    def set_callbacks(self, 
                      on_state_update: Optional[Callable] = None,
                      on_playback_progress: Optional[Callable] = None,
                      on_error: Optional[Callable] = None):
        """设置回调函数"""
        self._on_state_update = on_state_update
        self._on_playback_progress = on_playback_progress
        self._on_error = on_error

    # ==================== 内部工具 ====================

    def _init_joint_limits(self):
        """根据配置初始化每个电机的软关节限位"""
        try:
            left_cfg = JOINT_LIMITS.get("left", {})
            right_cfg = JOINT_LIMITS.get("right", {})

            for idx, mid in enumerate(LEFT_MOTOR_IDS):
                mins = left_cfg.get("position_min") or []
                maxs = left_cfg.get("position_max") or []
                min_v = float(mins[idx]) if idx < len(mins) else -3.14
                max_v = float(maxs[idx]) if idx < len(maxs) else 3.14
                self.joint_limits["left"][mid] = {"min": min_v, "max": max_v}

            for idx, mid in enumerate(RIGHT_MOTOR_IDS):
                mins = right_cfg.get("position_min") or []
                maxs = right_cfg.get("position_max") or []
                min_v = float(mins[idx]) if idx < len(mins) else -3.14
                max_v = float(maxs[idx]) if idx < len(maxs) else 3.14
                self.joint_limits["right"][mid] = {"min": min_v, "max": max_v}
        except Exception as e:
            # 出现异常时回退到对称范围
            print(f"[GroupController] 初始化关节限位失败，使用默认值: {e}")
            for mid in LEFT_MOTOR_IDS:
                self.joint_limits["left"][mid] = {"min": -3.14, "max": 3.14}
            for mid in RIGHT_MOTOR_IDS:
                self.joint_limits["right"][mid] = {"min": -3.14, "max": 3.14}

    def _get_joint_limit(self, arm_id: str, motor_id: int) -> Optional[Dict[str, float]]:
        """获取单个关节的软限位"""
        arm_limits = self.joint_limits.get(arm_id)
        if not arm_limits:
            return None
        return arm_limits.get(motor_id)
    
    def _emit_error(self, message: str):
        """发送错误消息"""
        print(f"[GroupController Error] {message}")
        if self._on_error:
            try:
                self._on_error(message)
            except Exception:
                pass
    
    # ==================== 连接管理 ====================
    
    def connect(self, arm_id: str, can_channel: Optional[str] = None) -> bool:
        """
        连接单个机械臂
        
        Args:
            arm_id: "left" 或 "right"
            can_channel: CAN通道名称，默认使用配置
        
        Returns:
            是否连接成功
        """
        if arm_id not in self.arms:
            self._emit_error(f"未知机械臂ID: {arm_id}")
            return False
        
        with self._lock:
            # 如果已连接，先断开
            if self.arms[arm_id] is not None:
                self.disconnect(arm_id)
            
            channel = can_channel or self.arm_states[arm_id].can_channel
            self.arm_states[arm_id].can_channel = channel
            
            try:
                print(f"[GroupController] 连接 {arm_id} 臂 ({channel})...")
                motor_ids = LEFT_MOTOR_IDS if arm_id == "left" else RIGHT_MOTOR_IDS
                controller = ArmController(
                    motor_ids=motor_ids,
                    can_channel=channel,
                    zero_offset_file=str(ZERO_OFFSET_FILE),
                )
                
                connected_ok = True
                if hasattr(controller, "connect"):
                    connected_ok = bool(controller.connect())
                
                if connected_ok:
                    self.arms[arm_id] = controller
                    self.arm_states[arm_id].connected = True
                    self.arm_states[arm_id].error = None
                    print(f"[GroupController] {arm_id} 臂连接成功")
                    return True
                else:
                    self.arm_states[arm_id].error = "连接失败"
                    self._emit_error(f"{arm_id} 臂连接失败")
                    return False
            except Exception as e:
                self.arm_states[arm_id].error = str(e)
                self._emit_error(f"{arm_id} 臂连接异常: {e}")
                return False
    
    def connect_all(self, left_channel: str = None, right_channel: str = None) -> Dict[str, bool]:
        """连接所有机械臂"""
        results = {}
        results["left"] = self.connect("left", left_channel)
        results["right"] = self.connect("right", right_channel)
        return results
    
    def disconnect(self, arm_id: str):
        """断开单个机械臂"""
        with self._lock:
            controller = self.arms.get(arm_id)
            if controller is not None:
                try:
                    # 先关闭电机
                    if hasattr(controller, 'enable_all'):
                        controller.enable_all(False)
                    
                    # 停止接收线程
                    if hasattr(controller, 'running'):
                        controller.running = False
                    
                    # 关闭CAN总线
                    if hasattr(controller, 'bus') and controller.bus is not None:
                        try:
                            controller.bus.shutdown()
                        except Exception:
                            pass
                except Exception as e:
                    print(f"[GroupController] 断开 {arm_id} 时出错: {e}")
                
                self.arms[arm_id] = None
            
            self.arm_states[arm_id].connected = False
            self.arm_states[arm_id].initialized = False
            print(f"[GroupController] {arm_id} 臂已断开")
    
    def disconnect_all(self):
        """断开所有机械臂"""
        self.disconnect("left")
        self.disconnect("right")
    
    # ==================== 初始化控制 ====================
    
    def init_arm(self, arm_id: str, mode: int = MODE_PROFILE_POSITION) -> bool:
        """初始化单个机械臂"""
        with self._lock:
            controller = self.arms.get(arm_id)
            if controller is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False
            
            try:
                controller.init_arm(mode)
                self.arm_states[arm_id].initialized = True
                # 初始化后设置为慢速，避免初始速度过快
                self.set_speed(
                    self.speed_params.get("velocity", 0.5),
                    self.speed_params.get("accel", 0.5),
                    self.speed_params.get("decel", 0.5),
                    arm_id=arm_id
                )
                return True
            except Exception as e:
                self.arm_states[arm_id].error = str(e)
                self._emit_error(f"{arm_id} 臂初始化失败: {e}")
                return False
    
    def init_target(self, mode: int = MODE_PROFILE_POSITION) -> Dict[str, bool]:
        """初始化当前控制目标"""
        results = {}
        
        if self.target in (ControlTarget.LEFT, ControlTarget.BOTH):
            results["left"] = self.init_arm("left", mode)
        
        if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH):
            results["right"] = self.init_arm("right", mode)
        
        return results
    
    def disable_arm(self, arm_id: Optional[str] = None):
        """关闭电机使能"""
        with self._lock:
            arms_to_disable = []
            
            if arm_id:
                if self.arms.get(arm_id):
                    arms_to_disable.append((arm_id, self.arms[arm_id]))
            else:
                # 根据当前目标决定
                if self.target in (ControlTarget.LEFT, ControlTarget.BOTH):
                    if self.arms.get("left"):
                        arms_to_disable.append(("left", self.arms["left"]))
                if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH):
                    if self.arms.get("right"):
                        arms_to_disable.append(("right", self.arms["right"]))
            
            for arm_name, controller in arms_to_disable:
                try:
                    if hasattr(controller, 'enable_all'):
                        controller.enable_all(False)
                    self.arm_states[arm_name].initialized = False
                    print(f"[GroupController] {arm_name} 臂电机已关闭")
                except Exception as e:
                    self._emit_error(f"{arm_name} 臂关闭失败: {e}")

    def deactivate_arm(self, arm_id: Optional[str] = None, target: Optional[str] = None):
        """去使能（保持连接，进入待命状态）"""
        with self._lock:
            if target:
                try:
                    self.target = ControlTarget(target)
                except Exception:
                    pass

            arms_to_deactivate = []
            if arm_id:
                if self.arms.get(arm_id):
                    arms_to_deactivate.append((arm_id, self.arms[arm_id]))
            else:
                # 根据当前目标决定
                if self.target in (ControlTarget.LEFT, ControlTarget.BOTH):
                    if self.arms.get("left"):
                        arms_to_deactivate.append(("left", self.arms["left"]))
                if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH):
                    if self.arms.get("right"):
                        arms_to_deactivate.append(("right", self.arms["right"]))

            for arm_name, controller in arms_to_deactivate:
                try:
                    if hasattr(controller, 'enable_all'):
                        controller.enable_all(False)  # 0x2A [0]
                    print(f"[GroupController] {arm_name} 臂已去使能")
                except Exception as e:
                    self._emit_error(f"{arm_name} 臂去使能失败: {e}")
    
    # ==================== 紧急控制 ====================
    
    def emergency_stop(self):
        """紧急停止所有机械臂"""
        print("[GroupController] !!! 紧急停止 !!!")
        
        # 先停止播放
        self.stop_playback()
        
        with self._lock:
            for arm_id, controller in self.arms.items():
                if controller is not None:
                    try:
                        controller.enable_all(False)
                        print(f"[GroupController] {arm_id} 臂已停止")
                    except Exception as e:
                        print(f"[GroupController] 停止 {arm_id} 臂时出错: {e}")
    
    def go_to_zero(self, arm_id: Optional[str] = None):
        """安全回零"""
        with self._lock:
            arms_to_control = []
            
            if arm_id:
                if self.arms.get(arm_id):
                    arms_to_control.append((arm_id, self.arms[arm_id]))
            else:
                # 根据当前目标决定
                if self.target in (ControlTarget.LEFT, ControlTarget.BOTH):
                    if self.arms.get("left"):
                        arms_to_control.append(("left", self.arms["left"]))
                if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH):
                    if self.arms.get("right"):
                        arms_to_control.append(("right", self.arms["right"]))
            
            for arm_name, controller in arms_to_control:
                try:
                    controller.go_to_zero()
                    print(f"[GroupController] {arm_name} 臂已回零")
                except Exception as e:
                    self._emit_error(f"{arm_name} 臂回零失败: {e}")
    
    # ==================== 状态获取 ====================
    
    def get_state(self) -> Dict[str, Any]:
        """获取完整状态"""
        with self._lock:
            state = {
                "target": self.target.value,
                "playback": {
                    "state": self.playback.state.value,
                    "trajectory_name": self.playback.trajectory_name,
                    "current_point": self.playback.current_point,
                    "total_points": self.playback.total_points,
                    "progress": self.playback.progress
                },
                "arms": {},
                "joint_limits": {"left": {}, "right": {}},
            }
            
            for arm_id in ["left", "right"]:
                arm_state = self.arm_states[arm_id]
                controller = self.arms.get(arm_id)
                
                motors_data = {}
                if controller is not None and hasattr(controller, 'motors'):
                    for mid, motor in controller.motors.items():
                        motors_data[mid] = {
                            "position": getattr(motor, 'position', 0.0),
                            "velocity": getattr(motor, 'velocity', 0.0),
                            "enabled": getattr(motor, 'enabled', False),
                            "mode": getattr(motor, 'mode', 0)
                        }
                
                # 组装关节状态
                state["arms"][arm_id] = {
                    "connected": arm_state.connected,
                    "initialized": arm_state.initialized,
                    "can_channel": arm_state.can_channel,
                    "error": arm_state.error,
                    "motors": motors_data
                }

                # 附加关节软限位信息
                for mid, limits in self.joint_limits.get(arm_id, {}).items():
                    state["joint_limits"][arm_id][str(mid)] = {
                        "min": limits["min"],
                        "max": limits["max"],
                    }
            
            return state
    
    def read_positions(self):
        """读取所有位置"""
        with self._lock:
            for controller in self.arms.values():
                if controller is not None:
                    try:
                        controller.read_all_positions()
                    except Exception:
                        pass
    
    # ==================== 手动控制 ====================
    
    def set_position(self, arm_id: str, motor_id: int, position: float) -> bool:
        """设置单个电机位置"""
        with self._lock:
            controller = self.arms.get(arm_id)
            if controller is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False

            # 软限位检查（防御性校验）
            limits = self._get_joint_limit(arm_id, motor_id)
            if limits is not None:
                if position < limits["min"] or position > limits["max"]:
                    self._emit_error(
                        f"{arm_id} 臂电机 {motor_id} 目标位置 {position:.4f} 超出软限位 "
                        f"[{limits['min']:.4f}, {limits['max']:.4f}]"
                    )
                    return False

            try:
                controller.set_position(motor_id, position)
                return True
            except Exception as e:
                self._emit_error(f"设置位置失败: {e}")
                return False

    def set_position_offset(self, arm_id: str, motor_id: int, position: float) -> bool:
        """设置相对于零点的位置"""
        with self._lock:
            controller = self.arms.get(arm_id)
            if controller is None:
                self._emit_error(f"{arm_id} 臂未连接")
                return False

            # 软限位检查：这里的 position 视为相对零点后的目标角度
            limits = self._get_joint_limit(arm_id, motor_id)
            if limits is not None:
                if position < limits["min"] or position > limits["max"]:
                    self._emit_error(
                        f"{arm_id} 臂电机 {motor_id} 相对零点目标位置 {position:.4f} 超出软限位 "
                        f"[{limits['min']:.4f}, {limits['max']:.4f}]"
                    )
                    return False

            try:
                controller.set_position_with_offset(motor_id, position)
                return True
            except Exception as e:
                self._emit_error(f"设置相对零点位置失败: {e}")
                return False
    
    def set_speed(self, velocity: float, accel: float, decel: float, arm_id: Optional[str] = None):
        """设置速度参数"""
        with self._lock:
            # 记录当前参数（用于回显）
            try:
                self.speed_params = {
                    "velocity": float(velocity),
                    "accel": float(accel),
                    "decel": float(decel),
                }
            except Exception:
                pass

            arms_to_control = []
            
            if arm_id:
                if self.arms.get(arm_id):
                    arms_to_control.append(self.arms[arm_id])
            else:
                for aid in ["left", "right"]:
                    if self.arms.get(aid):
                        arms_to_control.append(self.arms[aid])
            
            for controller in arms_to_control:
                try:
                    controller.set_all_profile_velocity(velocity)
                    controller.set_all_profile_acceleration(accel)
                    controller.set_all_profile_deceleration(decel)
                except Exception as e:
                    self._emit_error(f"设置速度失败: {e}")
    
    def set_speed_preset(self, preset: str, arm_id: Optional[str] = None):
        """设置速度预设"""
        if preset not in SPEED_PRESETS:
            self._emit_error(f"未知速度预设: {preset}")
            return
        
        params = SPEED_PRESETS[preset]
        self.set_speed(params["velocity"], params["accel"], params["decel"], arm_id)

    def get_speed_params(self) -> Dict[str, float]:
        """获取当前速度参数（用于前端显示）"""
        with self._lock:
            return {
                "velocity": float(self.speed_params.get("velocity", 1.0)),
                "accel": float(self.speed_params.get("accel", 1.0)),
                "decel": float(self.speed_params.get("decel", 1.0)),
            }
    
    # ==================== 自由拖动 ====================
    
    def enable_freedrive(self, arm_id: Optional[str] = None):
        """启用自由拖动模式"""
        with self._lock:
            arms = self._get_target_arms(arm_id)
            for aid, controller in arms:
                try:
                    controller.enable_freedrive()
                except Exception as e:
                    self._emit_error(f"{aid} 自由拖动启用失败: {e}")
    
    def disable_freedrive(self, arm_id: Optional[str] = None):
        """退出自由拖动模式（会自动重新使能电机）"""
        with self._lock:
            arms = self._get_target_arms(arm_id)
            for aid, controller in arms:
                try:
                    controller.disable_freedrive()  # 内部会调用 enable_all(True) 重新使能
                    print(f"[GroupController] {aid} 臂已退出自由拖动并重新使能")
                except Exception as e:
                    self._emit_error(f"{aid} 自由拖动退出失败: {e}")
    
    def _get_target_arms(self, arm_id: Optional[str] = None) -> List[tuple]:
        """获取目标机械臂列表"""
        arms = []
        if arm_id:
            if self.arms.get(arm_id):
                arms.append((arm_id, self.arms[arm_id]))
        else:
            if self.target in (ControlTarget.LEFT, ControlTarget.BOTH):
                if self.arms.get("left"):
                    arms.append(("left", self.arms["left"]))
            if self.target in (ControlTarget.RIGHT, ControlTarget.BOTH):
                if self.arms.get("right"):
                    arms.append(("right", self.arms["right"]))
        return arms
    
    # ==================== 零点标定 ====================
    
    def calibrate_zero(self, arm_id: Optional[str] = None) -> Dict[str, Any]:
        """标定零点"""
        with self._lock:
            arms = self._get_target_arms(arm_id)
            results = {}
            
            for aid, controller in arms:
                try:
                    if hasattr(controller, 'calibrate_zero'):
                        controller.calibrate_zero()
                        results[aid] = "success"
                        print(f"[GroupController] {aid} 臂零点标定完成")
                    else:
                        results[aid] = "not_supported"
                except Exception as e:
                    results[aid] = f"error: {e}"
                    self._emit_error(f"{aid} 臂零点标定失败: {e}")
            
            return results
    
    def get_zero_offsets(self) -> Dict[str, Dict]:
        """获取零点偏移"""
        with self._lock:
            offsets = {}
            
            for aid in ["left", "right"]:
                controller = self.arms.get(aid)
                if controller and hasattr(controller, 'zero_offsets'):
                    offsets[aid] = {str(k): float(v) for k, v in controller.zero_offsets.items()}
            
            return offsets
    
    # ==================== 示教功能 ====================
    
    def record_point(self, name: str = None, arm_id: Optional[str] = None) -> Dict[str, Any]:
        """记录当前位置点"""
        with self._lock:
            self.read_positions()
            time.sleep(0.3)
            
            point = {
                "name": name or f"point_{int(time.time())}",
                "timestamp": time.time(),
                "arms": {}
            }
            
            arms = self._get_target_arms(arm_id)
            for aid, controller in arms:
                positions = {}
                for mid in controller.motor_ids:
                    positions[str(mid)] = controller.motors[mid].position
                point["arms"][aid] = positions
            
            return point
    
    # ==================== 轨迹播放 ====================
    
    def load_trajectory(self, filename: str) -> Optional[Dict]:
        """加载轨迹文件"""
        filepath = TRAJECTORIES_DIR / filename
        if not filepath.exists():
            self._emit_error(f"轨迹文件不存在: {filename}")
            return None
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            self._emit_error(f"加载轨迹失败: {e}")
            return None
    
    def list_trajectories(self) -> List[str]:
        """列出所有轨迹文件"""
        if not TRAJECTORIES_DIR.exists():
            return []
        return [f.name for f in TRAJECTORIES_DIR.glob("*.json")]
    
    def play_trajectory(self, filename: str, sync: bool = True, loop_override=None):
        """
        播放轨迹
        
        Args:
            filename: 轨迹文件名
            sync: 是否同步播放（双臂同时开始）
            loop_override: 循环播放覆盖 (None=使用文件设置, True/False=覆盖)
        """
        if self.playback.state == PlaybackState.PLAYING:
            self._emit_error("已有轨迹正在播放")
            return
        
        trajectory = self.load_trajectory(filename)
        if trajectory is None:
            return
        
        points = trajectory.get("points", [])
        if not points:
            self._emit_error("轨迹文件没有轨迹点")
            return
        
        # 循环设置：前端覆盖 > 文件内设置
        loop = loop_override if loop_override is not None else trajectory.get("loop", False)
        print(f"[GroupController] play_trajectory: loop_override={loop_override!r}, file_loop={trajectory.get('loop')!r}, final_loop={loop!r}")
        
        self._stop_playback.clear()
        self._pause_playback.clear()
        
        self.playback.state = PlaybackState.PLAYING
        self.playback.trajectory_name = trajectory.get("name", filename)
        self.playback.current_point = 0
        self.playback.total_points = len(points)
        self.playback.progress = 0.0
        
        # 在后台线程中播放
        self._playback_thread = threading.Thread(
            target=self._playback_worker,
            args=(points, loop, trajectory.get("speed_multiplier", 1.0)),
            daemon=True
        )
        self._playback_thread.start()
    
    def _playback_worker(self, points: List[Dict], loop: bool, speed_mult: float):
        """轨迹播放工作线程"""
        print(f"[GroupController] _playback_worker started: loop={loop!r}, points={len(points)}, speed={speed_mult}")
        try:
            while True:
                for i, point in enumerate(points):
                    # 检查停止信号
                    if self._stop_playback.is_set():
                        break
                    
                    # 检查暂停信号
                    while self._pause_playback.is_set():
                        if self._stop_playback.is_set():
                            break
                        time.sleep(0.1)
                    
                    if self._stop_playback.is_set():
                        break
                    
                    # 更新进度
                    self.playback.current_point = i + 1
                    self.playback.progress = (i + 1) / len(points) * 100
                    
                    # 执行移动
                    self._execute_point(point)
                    
                    # 回调进度更新
                    if self._on_playback_progress:
                        try:
                            self._on_playback_progress({
                                "current": i + 1,
                                "total": len(points),
                                "progress": self.playback.progress,
                                "point_name": point.get("name", "")
                            })
                        except Exception:
                            pass
                    
                    # 等待延时
                    delay = point.get("delay", 1.0) / speed_mult
                    time.sleep(delay)
                
                print(f"[GroupController] loop iteration done: loop={loop!r}, stop_set={self._stop_playback.is_set()}")
                if not loop or self._stop_playback.is_set():
                    break
                # 循环重置进度
                self.playback.current_point = 0
                self.playback.progress = 0.0
        
        except Exception as e:
            self._emit_error(f"轨迹播放出错: {e}")
        
        finally:
            self.playback.state = PlaybackState.IDLE
            self.playback.progress = 100.0 if not self._stop_playback.is_set() else self.playback.progress
    
    def _execute_point(self, point: Dict):
        """执行单个轨迹点"""
        positions = point.get("positions", {})
        
        # 获取要控制的机械臂
        arms = self._get_target_arms()
        
        # 顺序执行双臂运动（避免CAN总线冲突）
        for arm_id, controller in arms:
            self._move_arm_to_positions(controller, positions)
    
    def _move_arm_to_positions(self, controller: ArmController, positions: Dict):
        """移动机械臂到指定位置"""
        try:
            for mid_str, pos in positions.items():
                mid = int(mid_str)
                if mid in controller.motor_ids:
                    controller.set_position(mid, pos)
                    time.sleep(0.01)  # 短暂延时避免CAN总线拥塞
        except Exception as e:
            print(f"[GroupController] 移动位置出错: {e}")
    
    def pause_playback(self):
        """暂停播放"""
        if self.playback.state == PlaybackState.PLAYING:
            self._pause_playback.set()
            self.playback.state = PlaybackState.PAUSED
    
    def resume_playback(self):
        """恢复播放"""
        if self.playback.state == PlaybackState.PAUSED:
            self._pause_playback.clear()
            self.playback.state = PlaybackState.PLAYING
    
    def stop_playback(self):
        """停止播放"""
        self._stop_playback.set()
        self._pause_playback.clear()
        self.playback.state = PlaybackState.STOPPING
        
        if self._playback_thread and self._playback_thread.is_alive():
            self._playback_thread.join(timeout=2.0)
        
        self.playback.state = PlaybackState.IDLE
    
    # ==================== 目标切换 ====================
    
    def set_target(self, target: str):
        """设置控制目标"""
        try:
            self.target = ControlTarget(target)
        except ValueError:
            self._emit_error(f"无效的控制目标: {target}")
    
    # ==================== 清理 ====================
    
    def cleanup(self):
        """清理资源"""
        self.stop_playback()
        self.disconnect_all()


# 单例模式
_controller_instance: Optional[GroupController] = None


def get_controller() -> GroupController:
    """获取控制器单例"""
    global _controller_instance
    if _controller_instance is None:
        _controller_instance = GroupController()
    return _controller_instance
