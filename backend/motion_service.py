"""
统一笛卡尔运动服务：协议适配、状态广播与执行控制。
"""

from __future__ import annotations

import math
import time
import uuid
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

import eventlet

from .motion_types import ErrorCode, ERROR_MESSAGES, Frame, MotionCommand, MotionState, MotionType


@dataclass
class _Runtime:
    command: MotionCommand
    sid: str
    created_at: float
    cancel_requested: bool = False
    pause_requested: bool = False
    paused_sent: bool = False
    completed: bool = False
    state: str = MotionState.ACCEPTED.value


class MotionService:
    def __init__(
        self,
        controller,
        cartesian,
        emit_fn: Callable[..., None],
        get_joints_fn: Callable[[Any, str], list],
    ):
        self.controller = controller
        self.cartesian = cartesian
        self.emit = emit_fn
        self.get_joints = get_joints_fn
        self._lock = threading.RLock()
        self._active_by_arm: Dict[str, _Runtime] = {}

    @staticmethod
    def _now_ms() -> int:
        return int(time.time() * 1000)

    def _emit(self, event: str, payload: Dict[str, Any], sid: Optional[str] = None):
        if sid:
            self.emit(event, payload, to=sid)
        self.emit(event, payload)

    def _mk_error(self, code: ErrorCode, detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return {
            "error_code": code.value,
            "message": ERROR_MESSAGES.get(code.value, code.value),
            "detail": detail or {},
            "server_ts": self._now_ms(),
        }

    def _emit_rejected(self, sid: str, command: MotionCommand, code: ErrorCode, detail: Optional[Dict[str, Any]] = None):
        payload = {
            "command_id": command.command_id,
            "arm_id": command.arm_id,
            "motion_type": command.motion_type.value,
            "state": "rejected",
            "deprecated": command.deprecated,
        }
        payload.update(self._mk_error(code, detail))
        self._emit("motion:rejected", payload, sid=sid)

    def _emit_status(self, rt: _Runtime, state: MotionState, progress: Optional[float] = None, extra: Optional[Dict[str, Any]] = None):
        payload = {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": state.value,
            "deprecated": rt.command.deprecated,
            "server_ts": self._now_ms(),
        }
        if progress is not None:
            payload["progress"] = max(0.0, min(1.0, float(progress)))
        if extra:
            payload.update(extra)

        if state == MotionState.ACCEPTED:
            self._emit("motion:accepted", payload, sid=rt.sid)
        elif state == MotionState.COMPLETED:
            self._emit("motion:completed", payload, sid=rt.sid)
        elif state == MotionState.CANCELLED:
            self._emit("motion:cancelled", payload, sid=rt.sid)
        elif state == MotionState.PAUSED:
            self._emit("motion:paused", payload, sid=rt.sid)
        else:
            self._emit("motion:status", payload, sid=rt.sid)
        rt.state = state.value

    def _emit_failed(self, rt: _Runtime, code: ErrorCode, detail: Optional[Dict[str, Any]] = None):
        payload = {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": MotionState.FAILED.value,
            "deprecated": rt.command.deprecated,
        }
        payload.update(self._mk_error(code, detail))
        self._emit("motion:failed", payload, sid=rt.sid)
        rt.state = MotionState.FAILED.value

    def _is_arm_ready(self, arm_id: str) -> bool:
        arm = self.controller.arms.get(arm_id)
        arm_state = self.controller.arm_states.get(arm_id)
        return bool(arm is not None and arm_state and arm_state.connected and arm_state.initialized)

    def _validate_constraints(self, constraints: Dict[str, Any]) -> bool:
        numeric_fields = (
            "max_velocity",
            "max_acceleration",
            "max_angular_velocity",
            "max_angular_acceleration",
        )
        for key in numeric_fields:
            if key in constraints:
                try:
                    val = float(constraints[key])
                except Exception:
                    return False
                if not math.isfinite(val) or val <= 0:
                    return False
        return True

    def _validate_target(self, cmd: MotionCommand) -> bool:
        if cmd.motion_type in (MotionType.CARTESIAN_PTP, MotionType.CARTESIAN_LINEAR):
            pose = cmd.target.get("pose") if isinstance(cmd.target, dict) else None
            if not isinstance(pose, dict):
                return False
            for k in ("x", "y", "z"):
                if k not in pose:
                    return False
                try:
                    val = float(pose.get(k))
                except Exception:
                    return False
                if not math.isfinite(val):
                    return False
            # 姿态字段可选；若提供需为数值
            for k in ("rx", "ry", "rz"):
                if k in pose and pose.get(k) is not None:
                    try:
                        val = float(pose.get(k))
                    except Exception:
                        return False
                    if not math.isfinite(val):
                        return False
            return True

        if cmd.motion_type == MotionType.CARTESIAN_JOG:
            delta = cmd.target.get("delta") if isinstance(cmd.target, dict) else None
            if not isinstance(delta, dict):
                return False
            keys = ("dx", "dy", "dz", "drx", "dry", "drz")
            has_any = False
            for k in keys:
                if k in delta:
                    has_any = True
                    try:
                        val = float(delta.get(k, 0.0))
                    except Exception:
                        return False
                    if not math.isfinite(val):
                        return False
            return has_any

        return False

    def _make_command(self, payload: Dict[str, Any], legacy_event: Optional[str] = None, deprecated: bool = False) -> MotionCommand:
        motion_type = MotionType(payload.get("motion_type", MotionType.CARTESIAN_PTP.value))
        frame = Frame(payload.get("frame", Frame.BASE.value))
        command_id = payload.get("command_id") or str(uuid.uuid4())
        target = payload.get("target") or {}
        constraints = payload.get("constraints") or {}
        options = payload.get("options") or {}
        return MotionCommand(
            command_id=command_id,
            arm_id=payload.get("arm_id", "left"),
            motion_type=motion_type,
            frame=frame,
            target=target,
            constraints=constraints,
            options=options,
            legacy_event=legacy_event,
            deprecated=deprecated,
        )

    def execute(
        self,
        sid: str,
        payload: Dict[str, Any],
        legacy_event: Optional[str] = None,
        deprecated: bool = False,
        allow_preempt: bool = False,
    ):
        cmd = self._make_command(payload, legacy_event=legacy_event, deprecated=deprecated)

        if cmd.arm_id not in ("left", "right"):
            self._emit_rejected(sid, cmd, ErrorCode.CONSTRAINT_VIOLATION, {"arm_id": cmd.arm_id})
            return
        if not self._validate_constraints(cmd.constraints):
            self._emit_rejected(sid, cmd, ErrorCode.CONSTRAINT_VIOLATION, {"constraints": cmd.constraints})
            return
        if not self._validate_target(cmd):
            self._emit_rejected(sid, cmd, ErrorCode.CONSTRAINT_VIOLATION, {"target": cmd.target})
            return
        if not self._is_arm_ready(cmd.arm_id):
            self._emit_rejected(sid, cmd, ErrorCode.ARM_NOT_CONNECTED)
            return

        with self._lock:
            active = self._active_by_arm.get(cmd.arm_id)
            if active and not active.completed:
                if allow_preempt:
                    active.cancel_requested = True
                    self._emit_failed(active, ErrorCode.COMMAND_PREEMPTED, {"preempted_by": cmd.command_id})
                else:
                    self._emit_rejected(sid, cmd, ErrorCode.COMMAND_IN_PROGRESS, {"active_command_id": active.command.command_id})
                    return

            rt = _Runtime(command=cmd, sid=sid, created_at=time.time())
            self._active_by_arm[cmd.arm_id] = rt

        self._emit_status(rt, MotionState.ACCEPTED)
        eventlet.spawn_n(self._run_command, rt)

    def _wait_if_paused(self, rt: _Runtime):
        while rt.pause_requested and not rt.cancel_requested:
            if not rt.paused_sent:
                self._emit_status(rt, MotionState.PAUSED)
                rt.paused_sent = True
            eventlet.sleep(0.05)
        if rt.paused_sent and not rt.cancel_requested:
            self._emit("motion:resumed", {
                "command_id": rt.command.command_id,
                "arm_id": rt.command.arm_id,
                "motion_type": rt.command.motion_type.value,
                "state": "resumed",
                "deprecated": rt.command.deprecated,
                "server_ts": self._now_ms(),
            }, sid=rt.sid)
            rt.paused_sent = False

    def _complete_and_cleanup(self, rt: _Runtime):
        rt.completed = True
        with self._lock:
            active = self._active_by_arm.get(rt.command.arm_id)
            if active and active.command.command_id == rt.command.command_id:
                self._active_by_arm.pop(rt.command.arm_id, None)

    def _run_command(self, rt: _Runtime):
        try:
            self._emit_status(rt, MotionState.PLANNING, progress=0.0)
            if rt.command.motion_type == MotionType.CARTESIAN_PTP:
                ok, detail = self._run_ptp(rt)
            elif rt.command.motion_type == MotionType.CARTESIAN_LINEAR:
                ok, detail = self._run_linear(rt)
            else:
                ok, detail = self._run_jog(rt)

            if rt.cancel_requested:
                self._emit_status(rt, MotionState.CANCELLED)
                return
            if not ok:
                self._emit_failed(rt, detail.get("error_code", ErrorCode.INTERNAL_ERROR), detail.get("detail"))
                return

            duration_ms = int((time.time() - rt.created_at) * 1000)
            payload = {
                "duration_ms": duration_ms,
                "final_error_mm": detail.get("final_error_mm", 0.0),
            }
            self._emit_status(rt, MotionState.COMPLETED, progress=1.0, extra=payload)

            if rt.command.legacy_event in ("cartesian_move_to", "smooth_cartesian_move"):
                self._emit("cartesian:moved", {
                    "arm_id": rt.command.arm_id,
                    "x": detail.get("target_pose", {}).get("x"),
                    "y": detail.get("target_pose", {}).get("y"),
                    "z": detail.get("target_pose", {}).get("z"),
                    "error_mm": detail.get("final_error_mm", 0.0),
                    "deprecated": True,
                }, sid=rt.sid)
            elif rt.command.legacy_event == "cartesian_jog":
                self._emit("cartesian:jogged", {
                    "arm_id": rt.command.arm_id,
                    "new_pos": detail.get("new_pos"),
                    "error_mm": detail.get("final_error_mm", 0.0),
                    "deprecated": True,
                }, sid=rt.sid)
        except Exception as e:
            self._emit_failed(rt, ErrorCode.INTERNAL_ERROR, {"exception": str(e)})
        finally:
            self._complete_and_cleanup(rt)

    def _ik_or_error(self, arm_id: str, pose: Dict[str, float], current_joints: list):
        result = self.cartesian.compute_ik(
            arm_id,
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            float(pose.get("z", 0.0)),
            current_joints,
            roll=pose.get("rx"),
            pitch=pose.get("ry"),
            yaw=pose.get("rz"),
        )
        if result is None:
            return None, ErrorCode.IK_UNREACHABLE
        if float(result.get("error_mm", 999.0)) > 5.0:
            return None, ErrorCode.IK_LOW_ACCURACY
        return result, None

    def _run_ptp(self, rt: _Runtime):
        arm_ctrl = self.controller.arms.get(rt.command.arm_id)
        current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
        pose = rt.command.target.get("pose") or {}

        ik_result, err = self._ik_or_error(rt.command.arm_id, pose, current_joints)
        if err:
            return False, {"error_code": err, "detail": {"pose": pose}}

        self._emit_status(rt, MotionState.RUNNING, progress=0.2)
        self._wait_if_paused(rt)
        if rt.cancel_requested:
            return False, {"error_code": ErrorCode.COMMAND_PREEMPTED}

        ok = self.controller.set_joint_offsets(rt.command.arm_id, ik_result["joints"])
        if not ok:
            return False, {"error_code": ErrorCode.JOINT_LIMIT_REJECTED}

        return True, {"final_error_mm": float(ik_result["error_mm"]), "target_pose": pose}

    def _run_linear(self, rt: _Runtime):
        arm_ctrl = self.controller.arms.get(rt.command.arm_id)
        current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
        current_fk = self.cartesian.compute_fk(rt.command.arm_id, current_joints)
        pose = rt.command.target.get("pose") or {}

        start = {
            "x": float(current_fk["x"]),
            "y": float(current_fk["y"]),
            "z": float(current_fk["z"]),
            "rx": float(current_fk["roll"]),
            "ry": float(current_fk["pitch"]),
            "rz": float(current_fk["yaw"]),
        }
        target = {
            "x": float(pose.get("x", start["x"])),
            "y": float(pose.get("y", start["y"])),
            "z": float(pose.get("z", start["z"])),
            "rx": float(pose.get("rx", start["rx"])),
            "ry": float(pose.get("ry", start["ry"])),
            "rz": float(pose.get("rz", start["rz"])),
        }

        steps = 20
        joints_seed = current_joints
        final_error = 0.0
        self._emit_status(rt, MotionState.RUNNING, progress=0.05)

        for i in range(steps):
            if rt.cancel_requested:
                break
            self._wait_if_paused(rt)
            if rt.cancel_requested:
                break

            t = float(i + 1) / float(steps)
            interp = {k: start[k] + (target[k] - start[k]) * t for k in start.keys()}
            ik_result, err = self._ik_or_error(rt.command.arm_id, interp, joints_seed)
            if err:
                return False, {"error_code": err, "detail": {"step": i + 1, "target_pose": interp}}

            ok = self.controller.set_joint_offsets(rt.command.arm_id, ik_result["joints"])
            if not ok:
                return False, {"error_code": ErrorCode.JOINT_LIMIT_REJECTED, "detail": {"step": i + 1}}

            joints_seed = ik_result["joints"]
            final_error = float(ik_result["error_mm"])
            progress = 0.05 + 0.9 * t
            self._emit_status(
                rt,
                MotionState.RUNNING,
                progress=progress,
                extra={
                    "pose_error_mm": final_error,
                    "joint_error_rad": 0.0,
                    "current_pose": {
                        "x": interp["x"],
                        "y": interp["y"],
                        "z": interp["z"],
                        "rx": interp["rx"],
                        "ry": interp["ry"],
                        "rz": interp["rz"],
                    },
                },
            )
            eventlet.sleep(0.02)

        return True, {"final_error_mm": final_error, "target_pose": target}

    def _run_jog(self, rt: _Runtime):
        arm_ctrl = self.controller.arms.get(rt.command.arm_id)
        current_joints = self.get_joints(arm_ctrl, rt.command.arm_id)
        delta = rt.command.target.get("delta") or {}

        dx = float(delta.get("dx", 0.0))
        dy = float(delta.get("dy", 0.0))
        dz = float(delta.get("dz", 0.0))
        drx = float(delta.get("drx", 0.0))
        dry = float(delta.get("dry", 0.0))
        drz = float(delta.get("drz", 0.0))

        current_fk = self.cartesian.compute_fk(rt.command.arm_id, current_joints)
        target_pose = {
            "x": float(current_fk["x"]) + dx,
            "y": float(current_fk["y"]) + dy,
            "z": float(current_fk["z"]) + dz,
            "rx": float(current_fk["roll"]) + drx,
            "ry": float(current_fk["pitch"]) + dry,
            "rz": float(current_fk["yaw"]) + drz,
        }

        ik_result, err = self._ik_or_error(rt.command.arm_id, target_pose, current_joints)
        if err:
            return False, {"error_code": err, "detail": {"target_pose": target_pose}}

        self._emit_status(rt, MotionState.RUNNING, progress=0.3)
        self._wait_if_paused(rt)
        if rt.cancel_requested:
            return False, {"error_code": ErrorCode.COMMAND_PREEMPTED}

        ok = self.controller.set_joint_offsets(rt.command.arm_id, ik_result["joints"])
        if not ok:
            return False, {"error_code": ErrorCode.JOINT_LIMIT_REJECTED}

        return True, {
            "final_error_mm": float(ik_result["error_mm"]),
            "new_pos": {"x": target_pose["x"], "y": target_pose["y"], "z": target_pose["z"]},
            "target_pose": target_pose,
        }

    def _resolve_runtime(self, command_id: Optional[str], arm_id: Optional[str]) -> Optional[_Runtime]:
        with self._lock:
            if arm_id in ("left", "right"):
                rt = self._active_by_arm.get(arm_id)
                if not rt:
                    return None
                if command_id and rt.command.command_id != command_id:
                    return None
                return rt
            if command_id:
                for rt in self._active_by_arm.values():
                    if rt.command.command_id == command_id:
                        return rt
            return None

    def cancel(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            cmd = self._make_command({"command_id": payload.get("command_id") or str(uuid.uuid4()), "arm_id": payload.get("arm_id", "left")})
            self._emit_rejected(sid, cmd, ErrorCode.COMMAND_NOT_FOUND)
            return
        rt.cancel_requested = True
        self._emit("motion:status", {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": "cancelling",
            "server_ts": self._now_ms(),
        }, sid=sid)

    def pause(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            cmd = self._make_command({"command_id": payload.get("command_id") or str(uuid.uuid4()), "arm_id": payload.get("arm_id", "left")})
            self._emit_rejected(sid, cmd, ErrorCode.COMMAND_NOT_FOUND)
            return
        rt.pause_requested = True

    def resume(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            cmd = self._make_command({"command_id": payload.get("command_id") or str(uuid.uuid4()), "arm_id": payload.get("arm_id", "left")})
            self._emit_rejected(sid, cmd, ErrorCode.COMMAND_NOT_FOUND)
            return
        rt.pause_requested = False

    def query(self, sid: str, payload: Dict[str, Any]):
        rt = self._resolve_runtime(payload.get("command_id"), payload.get("arm_id"))
        if not rt:
            self._emit("motion:status", {"state": "idle", "server_ts": self._now_ms()}, sid=sid)
            return
        self._emit("motion:status", {
            "command_id": rt.command.command_id,
            "arm_id": rt.command.arm_id,
            "motion_type": rt.command.motion_type.value,
            "state": rt.state,
            "server_ts": self._now_ms(),
        }, sid=sid)

