"""
笛卡尔坐标控制器

基于 URDF 运动学模型提供正向运动学(FK)与逆向运动学(IK)，
支持绝对坐标控制和增量坐标控制。

坐标系: 以末端初始位姿(所有关节角=零时)为原点。
"""

import sys
import os
import math
import numpy as np
from typing import Dict, List, Optional, Tuple
from pathlib import Path

# ================================================================
# URDF 提取的运动学链 (来自 lens_dual_arm_description.urdf)
# ================================================================
# 每个关节: (parent→child平移xyz, 旋转轴xyz, 关节限位[lower, upper])
# 注意: 平移是父坐标系下 child 相对 parent 的偏移 (单位: 米)

LEFT_ARM_CHAIN = [
    # Fixed: Base_Link → Left_Shoulder_Base_Link
    {"type": "fixed", "xyz": [0, 0, 1.217], "axis": None},
    # J1: Shoulder Pitch (axis Y)
    {"type": "revolute", "xyz": [0, 0.096, 0], "axis": [0, 1, 0],
     "limits": [-2.97, 1.05]},
    # J2: Shoulder Roll (axis X)
    {"type": "revolute", "xyz": [0.0328, 0.075, 0], "axis": [1, 0, 0],
     "limits": [-0.35, 3.49]},
    # J3: Shoulder Yaw (axis Z)
    {"type": "revolute", "xyz": [-0.0328, 0, -0.12], "axis": [0, 0, 1],
     "limits": [-2.79, 2.79]},
    # J4: Elbow Pitch (axis Y)
    {"type": "revolute", "xyz": [0.0095, -0.0325, -0.15], "axis": [0, 1, 0],
     "limits": [-2.355, 0]},
    # J5: Wrist Yaw (axis Z)
    {"type": "revolute", "xyz": [-0.0095, 0.0325, -0.11], "axis": [0, 0, 1],
     "limits": [-2.79, 2.79]},
    # J6: Wrist Pitch (axis Y)
    {"type": "revolute", "xyz": [0, -0.022, -0.06], "axis": [0, 1, 0],
     "limits": [-1.57, 1.57]},
    # J7: Wrist Roll (axis X)
    {"type": "revolute", "xyz": [0.0225, 0.022, -0.064], "axis": [1, 0, 0],
     "limits": [-1.57, 1.57]},
]

RIGHT_ARM_CHAIN = [
    # Fixed: Base_Link → Right_Shoulder_Base_Link
    {"type": "fixed", "xyz": [0, 0, 1.217], "axis": None},
    # J1: Shoulder Pitch (axis Y)
    {"type": "revolute", "xyz": [0, -0.096, 0], "axis": [0, 1, 0],
     "limits": [-2.97, 1.05]},
    # J2: Shoulder Roll (axis X)
    {"type": "revolute", "xyz": [0.0328, -0.075, 0], "axis": [1, 0, 0],
     "limits": [-3.49, 0.35]},
    # J3: Shoulder Yaw (axis Z)
    {"type": "revolute", "xyz": [-0.0328, 0, -0.12], "axis": [0, 0, 1],
     "limits": [-2.79, 2.79]},
    # J4: Elbow Pitch (axis Y)
    {"type": "revolute", "xyz": [0.0095, -0.0325, -0.15], "axis": [0, 1, 0],
     "limits": [-2.355, 0]},
    # J5: Wrist Yaw (axis Z)
    {"type": "revolute", "xyz": [-0.0095, 0.0325, -0.11], "axis": [0, 0, 1],
     "limits": [-2.79, 2.79]},
    # J6: Wrist Pitch (axis Y)
    {"type": "revolute", "xyz": [0, 0.022, -0.06], "axis": [0, 1, 0],
     "limits": [-1.57, 1.57]},
    # J7: Wrist Roll (axis X)
    {"type": "revolute", "xyz": [0.0225, -0.022, -0.064], "axis": [1, 0, 0],
     "limits": [-1.57, 1.57]},
]

# 电机ID → 关节索引映射
LEFT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]
RIGHT_MOTOR_IDS = [61, 62, 63, 64, 65, 66, 67]


def _rotation_matrix(axis: List[float], angle: float) -> np.ndarray:
    """
    绕任意轴旋转的 3x3 旋转矩阵 (Rodrigues公式)
    
    Args:
        axis: 旋转轴 [x, y, z], 必须为单位向量
        angle: 旋转角度 (弧度)
    """
    ax, ay, az = axis[0], axis[1], axis[2]
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c

    return np.array([
        [t*ax*ax + c,     t*ax*ay - s*az,  t*ax*az + s*ay],
        [t*ax*ay + s*az,  t*ay*ay + c,     t*ay*az - s*ax],
        [t*ax*az - s*ay,  t*ay*az + s*ax,  t*az*az + c],
    ])


def _joint_transform(joint: dict, angle: float = 0.0) -> np.ndarray:
    """
    计算单个关节的 4x4 齐次变换矩阵
    
    T = Trans(xyz) * Rot(axis, angle)
    """
    T = np.eye(4)
    
    # 平移
    T[0, 3] = joint["xyz"][0]
    T[1, 3] = joint["xyz"][1]
    T[2, 3] = joint["xyz"][2]
    
    # 旋转 (仅对旋转关节)
    if joint["type"] == "revolute" and joint["axis"] is not None:
        R = _rotation_matrix(joint["axis"], angle)
        T[:3, :3] = R
    
    return T


def forward_kinematics(
    arm_id: str,
    joint_angles: List[float]
) -> Tuple[np.ndarray, np.ndarray]:
    """
    正向运动学: 关节角度 → 末端位姿
    
    Args:
        arm_id: "left" 或 "right"
        joint_angles: 7个关节角度 (弧度)
    
    Returns:
        (position, rotation_matrix): 末端位置 [x,y,z] 和 3x3 旋转矩阵
    """
    if len(joint_angles) != 7:
        raise ValueError(f"需要7个关节角度，实际{len(joint_angles)}个")
    
    chain = LEFT_ARM_CHAIN if arm_id == "left" else RIGHT_ARM_CHAIN
    
    T = np.eye(4)
    joint_idx = 0
    
    for joint_def in chain:
        if joint_def["type"] == "fixed":
            T_j = _joint_transform(joint_def, 0.0)
        else:
            T_j = _joint_transform(joint_def, joint_angles[joint_idx])
            joint_idx += 1
        T = T @ T_j
    
    position = T[:3, 3].copy()
    rotation = T[:3, :3].copy()
    
    return position, rotation


def forward_kinematics_all(
    arm_id: str,
    joint_angles: List[float]
) -> List[np.ndarray]:
    """
    正向运动学, 返回所有关节的变换矩阵 (用于雅可比计算)
    """
    chain = LEFT_ARM_CHAIN if arm_id == "left" else RIGHT_ARM_CHAIN
    
    T = np.eye(4)
    transforms = [T.copy()]
    joint_idx = 0
    
    for joint_def in chain:
        if joint_def["type"] == "fixed":
            T_j = _joint_transform(joint_def, 0.0)
        else:
            T_j = _joint_transform(joint_def, joint_angles[joint_idx])
            joint_idx += 1
        T = T @ T_j
        transforms.append(T.copy())
    
    return transforms


def compute_jacobian(
    arm_id: str,
    joint_angles: List[float]
) -> np.ndarray:
    """
    计算几何雅可比矩阵 (6x7)
    
    J = [Jv; Jw] 其中 Jv 是线速度雅可比, Jw 是角速度雅可比
    """
    chain = LEFT_ARM_CHAIN if arm_id == "left" else RIGHT_ARM_CHAIN
    transforms = forward_kinematics_all(arm_id, joint_angles)
    
    # 末端位置
    p_end = transforms[-1][:3, 3]
    
    J = np.zeros((6, 7))
    
    # 遍历旋转关节
    joint_idx = 0
    transform_idx = 0
    
    for joint_def in chain:
        transform_idx += 1
        if joint_def["type"] != "revolute":
            continue
        
        # 该关节在世界坐标系下的位置和旋转轴
        T_i = transforms[transform_idx - 1]  # 关节之前的变换
        
        # 关节的平移是在父坐标系下的, 需要计算关节所在位置
        p_i = transforms[transform_idx][:3, 3]
        
        # 旋转轴在世界坐标系下的方向
        axis_local = np.array(joint_def["axis"], dtype=float)
        R_i = transforms[transform_idx - 1][:3, :3]
        
        # 对于URDF: 旋转轴是在子坐标系平移之后、旋转之前的坐标系下定义的
        # 但由于平移不改变旋转轴方向, 可以用父坐标系的旋转来变换
        axis_world = R_i @ axis_local
        
        # 线速度雅可比: z_i × (p_end - p_i) 
        # 这里 p_i 应该是关节旋转中心的位置
        # 关节旋转中心 = 将关节平移应用到父坐标系后的位置
        joint_origin = T_i[:3, 3] + T_i[:3, :3] @ np.array(joint_def["xyz"])
        
        J[:3, joint_idx] = np.cross(axis_world, p_end - joint_origin)
        J[3:, joint_idx] = axis_world
        
        joint_idx += 1
    
    return J


def inverse_kinematics(
    arm_id: str,
    target_position: List[float],
    current_joints: List[float],
    max_iterations: int = 200,
    tolerance: float = 1e-4,
    damping: float = 0.1,
    orientation: Optional[np.ndarray] = None
) -> Optional[List[float]]:
    """
    逆向运动学: 末端位置 → 关节角度
    
    使用阻尼最小二乘法 (DLS / Levenberg-Marquardt)
    
    Args:
        arm_id: "left" 或 "right"
        target_position: 目标位置 [x, y, z] (基座坐标系)
        current_joints: 当前关节角度 (用作初始猜测)
        max_iterations: 最大迭代次数
        tolerance: 位置误差容差 (米)
        damping: 阻尼系数
        orientation: 目标姿态 (3x3旋转矩阵), None则只控制位置
    
    Returns:
        关节角度列表, 或 None (求解失败)
    """
    chain = LEFT_ARM_CHAIN if arm_id == "left" else RIGHT_ARM_CHAIN
    
    # 获取关节限位
    joint_limits = []
    for j in chain:
        if j["type"] == "revolute":
            joint_limits.append(j.get("limits", [-math.pi, math.pi]))
    
    q = np.array(current_joints, dtype=np.float64)
    target = np.array(target_position, dtype=np.float64)
    
    for iteration in range(max_iterations):
        # 当前末端位置
        pos, rot = forward_kinematics(arm_id, q.tolist())
        
        # 位置误差
        error = target - pos
        error_norm = np.linalg.norm(error)
        
        if error_norm < tolerance:
            return q.tolist()
        
        # 计算雅可比 (仅使用位置部分, 3x7)
        J_full = compute_jacobian(arm_id, q.tolist())
        J = J_full[:3, :]  # 只用线速度部分
        
        # 阻尼最小二乘: dq = J^T (J J^T + λ²I)^{-1} error
        JJT = J @ J.T
        lambda_sq = damping ** 2
        
        try:
            dq = J.T @ np.linalg.solve(JJT + lambda_sq * np.eye(3), error)
        except np.linalg.LinAlgError:
            return None
        
        # 限制步长
        max_step = 0.2  # 最大单步变化 (rad)
        step_size = np.max(np.abs(dq))
        if step_size > max_step:
            dq *= max_step / step_size
        
        # 更新关节角度
        q = q + dq
        
        # 应用关节限位
        for i in range(7):
            q[i] = np.clip(q[i], joint_limits[i][0], joint_limits[i][1])
    
    # 最终检查
    pos, _ = forward_kinematics(arm_id, q.tolist())
    if np.linalg.norm(target - pos) < tolerance * 10:
        return q.tolist()
    
    return None


class CartesianController:
    """
    笛卡尔坐标控制器
    
    提供以末端初始位姿为原点的坐标控制接口。
    坐标系原点 = 所有关节角为0时的末端执行器位置。
    """
    
    def __init__(self):
        # 计算左右臂的零位末端位置 (= 坐标系原点)
        self._origins = {}
        self._zero_rotations = {}
        
        for arm_id in ["left", "right"]:
            pos, rot = forward_kinematics(arm_id, [0.0] * 7)
            self._origins[arm_id] = pos.copy()
            self._zero_rotations[arm_id] = rot.copy()
        
        # 缓存当前关节角度 (初始为零位)
        self._current_joints = {
            "left": [0.0] * 7,
            "right": [0.0] * 7,
        }
    
    def set_current_joints(self, arm_id: str, joints: List[float]):
        """更新当前关节角度缓存"""
        self._current_joints[arm_id] = list(joints)
    
    def compute_fk(self, arm_id: str, joint_angles: List[float]) -> Dict:
        """
        计算FK, 返回末端原点坐标系下的位置
        
        Args:
            arm_id: "left" 或 "right"
            joint_angles: 7个关节角度 (弧度)
        
        Returns:
            {
                "x": float, "y": float, "z": float,  # 相对于末端原点 (米)
                "abs_x": float, "abs_y": float, "abs_z": float,  # 基座坐标系 (米)
            }
        """
        pos, rot = forward_kinematics(arm_id, joint_angles)
        origin = self._origins[arm_id]
        
        # 相对于末端原点的坐标
        relative = pos - origin
        
        return {
            "x": float(relative[0]),
            "y": float(relative[1]),
            "z": float(relative[2]),
            "abs_x": float(pos[0]),
            "abs_y": float(pos[1]),
            "abs_z": float(pos[2]),
        }
    
    def compute_ik(
        self,
        arm_id: str,
        x: float, y: float, z: float,
        current_joints: Optional[List[float]] = None
    ) -> Optional[Dict]:
        """
        计算IK: 末端原点坐标 → 关节角度
        
        Args:
            arm_id: "left" 或 "right"
            x, y, z: 末端原点坐标系下的目标位置 (米)
            current_joints: 当前关节角度 (用作IK初始猜测)
        
        Returns:
            {"joints": [7个弧度], "error_mm": 位置误差} 或 None
        """
        origin = self._origins[arm_id]
        
        # 转为基座坐标系
        target = origin + np.array([x, y, z])
        
        seed = current_joints or self._current_joints[arm_id]
        
        result = inverse_kinematics(
            arm_id, target.tolist(), seed
        )
        
        if result is None:
            return None
        
        # 验证
        pos, _ = forward_kinematics(arm_id, result)
        error = np.linalg.norm(target - pos) * 1000  # mm
        
        return {
            "joints": result,
            "error_mm": float(error),
        }
    
    def compute_ik_delta(
        self,
        arm_id: str,
        dx: float, dy: float, dz: float,
        current_joints: Optional[List[float]] = None
    ) -> Optional[Dict]:
        """
        增量IK: 当前位置 + (dx, dy, dz) → 新关节角度
        
        Args:
            arm_id: "left" 或 "right"
            dx, dy, dz: 增量位移 (米)
            current_joints: 当前关节角度
        
        Returns:
            {"joints": [7个弧度], "error_mm": 位置误差,
             "new_pos": {"x","y","z"}} 或 None
        """
        seed = current_joints or self._current_joints[arm_id]
        
        # 当前末端位置 (基座坐标系)
        current_pos, _ = forward_kinematics(arm_id, seed)
        
        # 新目标位置
        target = current_pos + np.array([dx, dy, dz])
        
        result = inverse_kinematics(
            arm_id, target.tolist(), seed
        )
        
        if result is None:
            return None
        
        # 验证并计算末端原点坐标
        pos, _ = forward_kinematics(arm_id, result)
        error = np.linalg.norm(target - pos) * 1000
        
        origin = self._origins[arm_id]
        relative = pos - origin
        
        return {
            "joints": result,
            "error_mm": float(error),
            "new_pos": {
                "x": float(relative[0]),
                "y": float(relative[1]),
                "z": float(relative[2]),
            },
        }
    
    def get_joint_limits(self, arm_id: str) -> List[Tuple[float, float]]:
        """获取关节限位"""
        chain = LEFT_ARM_CHAIN if arm_id == "left" else RIGHT_ARM_CHAIN
        limits = []
        for j in chain:
            if j["type"] == "revolute":
                limits.append(tuple(j.get("limits", [-math.pi, math.pi])))
        return limits
    
    def get_origin(self, arm_id: str) -> Dict:
        """获取坐标系原点 (基座坐标系下)"""
        origin = self._origins[arm_id]
        return {
            "x": float(origin[0]),
            "y": float(origin[1]),
            "z": float(origin[2]),
        }


# 单例
_instance: Optional[CartesianController] = None

def get_cartesian_controller() -> CartesianController:
    """获取笛卡尔控制器单例"""
    global _instance
    if _instance is None:
        _instance = CartesianController()
    return _instance
