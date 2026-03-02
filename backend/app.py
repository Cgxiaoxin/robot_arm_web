"""
机械臂演出控制系统 - Web服务主程序

Flask + Socket.IO 实现实时控制
"""
import os
import sys
import time
import threading
from pathlib import Path

from flask import Flask, send_from_directory, jsonify, request
import socketio
import eventlet

# 兼容脚本直接运行与模块运行
if __package__ is None or __package__ == "":
    project_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(project_root))
    from robot_arm_web.backend.config import (
        HOST, PORT, DEBUG, STATE_UPDATE_INTERVAL, TRAJECTORIES_DIR
    )
    from robot_arm_web.backend.group_controller import get_controller, ControlTarget
else:
    from .config import HOST, PORT, DEBUG, STATE_UPDATE_INTERVAL, TRAJECTORIES_DIR
    from .group_controller import get_controller, ControlTarget

# 创建Flask应用
BASE_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = BASE_DIR.parent / "frontend"
app = Flask(__name__, static_folder=str(FRONTEND_DIR), static_url_path='')

# 创建Socket.IO服务器
sio = socketio.Server(
    async_mode='eventlet',
    cors_allowed_origins='*',
    logger=DEBUG,
    engineio_logger=DEBUG
)
flask_app = socketio.WSGIApp(sio, app)

# 获取控制器实例
controller = get_controller()

# 状态更新定时器
_state_update_running = False


# ==================== 静态文件路由 ====================

@app.route('/')
def index():
    """主页"""
    return send_from_directory(str(FRONTEND_DIR), 'index.html')


@app.route('/<path:path>')
def static_files(path):
    """静态文件"""
    return send_from_directory(str(FRONTEND_DIR), path)


# ==================== REST API ====================

@app.route('/api/health')
def health():
    """健康检查"""
    return jsonify({"ok": True, "timestamp": time.time()})


@app.route('/api/trajectories')
def list_trajectories():
    """列出所有轨迹文件"""
    files = controller.list_trajectories()
    return jsonify({"trajectories": files})


@app.route('/api/state')
def get_state():
    """获取当前状态（HTTP方式，用于调试）"""
    return jsonify(controller.get_state())


# ==================== Socket.IO 事件处理 ====================

@sio.event
def connect(sid, environ):
    """客户端连接"""
    print(f"[Socket.IO] 客户端连接: {sid}")
    # 发送当前状态
    sio.emit('state:update', controller.get_state(), to=sid)


@sio.event
def disconnect(sid):
    """客户端断开"""
    print(f"[Socket.IO] 客户端断开: {sid}")


# ---------- 连接控制 ----------

@sio.event
def arm_connect(sid, data):
    """
    连接机械臂
    
    data: {
        "arm_id": "left" | "right" | "both",
        "left_channel": "can0",  # 可选
        "right_channel": "can1"  # 可选
    }
    """
    arm_id = data.get("arm_id", "both")
    left_ch = data.get("left_channel")
    right_ch = data.get("right_channel")
    
    try:
        if arm_id == "both":
            results = controller.connect_all(left_ch, right_ch)
            success = all(results.values())
        else:
            success = controller.connect(arm_id, left_ch if arm_id == "left" else right_ch)
            results = {arm_id: success}
        
        sio.emit('arm:connected', {"success": success, "results": results}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"连接失败: {str(e)}"}, to=sid)


@sio.event
def arm_disconnect(sid, data):
    """
    断开机械臂
    
    data: {"arm_id": "left" | "right" | "both"}
    """
    arm_id = data.get("arm_id", "both")
    
    try:
        if arm_id == "both":
            controller.disconnect_all()
        else:
            controller.disconnect(arm_id)
        
        sio.emit('arm:disconnected', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"断开失败: {str(e)}"}, to=sid)


# ---------- 初始化控制 ----------

@sio.event
def arm_init(sid, data):
    """
    初始化并使能机械臂
    
    data: {"arm_id": "left" | "right" | null, "mode": 6}
    """
    arm_id = data.get("arm_id")
    mode = data.get("mode", 6)  # 默认轨迹位置模式
    
    try:
        if arm_id:
            success = controller.init_arm(arm_id, mode)
            results = {arm_id: success}
        else:
            results = controller.init_target(mode)
        
        sio.emit('arm:initialized', {"results": results}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"启动使能失败: {str(e)}"}, to=sid)


@sio.event
def arm_deactivate(sid, data=None):
    """
    去使能（电机进入待命状态，可手动移动）
    
    data: {"arm_id": "left" | "right" | null, "target": "left" | "right" | "both"}
    """
    arm_id = data.get("arm_id") if data else None
    target = data.get("target") if data else None
    
    try:
        controller.deactivate_arm(arm_id, target)
        sio.emit('arm:deactivated', {"arm_id": arm_id or "all"}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"去使能失败: {str(e)}"}, to=sid)


@sio.event
def arm_disable(sid, data=None):
    """
    关闭电机使能
    
    data: {"arm_id": "left" | "right" | null}
    """
    arm_id = data.get("arm_id") if data else None
    
    try:
        controller.disable_arm(arm_id)
        sio.emit('arm:disabled', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"关闭电机失败: {str(e)}"}, to=sid)


# ---------- 紧急控制 ----------

@sio.event
def emergency_stop(sid, data=None):
    """紧急停止"""
    try:
        controller.emergency_stop()
        sio.emit('arm:emergency_stopped', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"急停失败: {str(e)}"}, to=sid)


@sio.event
def go_to_zero(sid, data):
    """
    安全回零
    
    data: {"arm_id": "left" | "right" | null}
    """
    arm_id = data.get("arm_id")
    
    try:
        controller.go_to_zero(arm_id)
        sio.emit('arm:went_to_zero', {"arm_id": arm_id}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"回零失败: {str(e)}"}, to=sid)


# ---------- 目标切换 ----------

@sio.event
def set_target(sid, data):
    """
    设置控制目标
    
    data: {"target": "left" | "right" | "both"}
    """
    target = data.get("target", "both")
    
    try:
        controller.set_target(target)
        sio.emit('target:changed', {"target": target}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"切换目标失败: {str(e)}"}, to=sid)


# ---------- 手动控制 ----------

@sio.event
def manual_set_position(sid, data):
    """
    手动设置位置
    
    data: {"arm_id": "left" | "right", "motor_id": 51, "position": 0.5}
    """
    arm_id = data.get("arm_id")
    motor_id = data.get("motor_id")
    position = data.get("position")
    
    if not all([arm_id, motor_id is not None, position is not None]):
        sio.emit('error', {"message": "缺少参数"}, to=sid)
        return
    
    try:
        success = controller.set_position(arm_id, motor_id, position)
        sio.emit('manual:position_set', {"success": success}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"设置位置失败: {str(e)}"}, to=sid)


@sio.event
def manual_set_position_offset(sid, data):
    """
    手动设置相对零点位置
    
    data: {"arm_id": "left" | "right", "motor_id": 51, "position": 0.0}
    """
    arm_id = data.get("arm_id")
    motor_id = data.get("motor_id")
    position = data.get("position")
    
    if not all([arm_id, motor_id is not None, position is not None]):
        sio.emit('error', {"message": "缺少参数"}, to=sid)
        return
    
    try:
        success = controller.set_position_offset(arm_id, motor_id, position)
        sio.emit('manual:position_set', {"success": success, "offset": True}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"设置相对零点位置失败: {str(e)}"}, to=sid)


@sio.event
def set_speed(sid, data):
    """
    设置速度参数
    
    data: {"velocity": 1.0, "accel": 1.0, "decel": 1.0, "arm_id": null}
    或: {"preset": "slow" | "medium" | "fast", "arm_id": null}
    """
    preset = data.get("preset")
    arm_id = data.get("arm_id")
    
    try:
        if preset:
            controller.set_speed_preset(preset, arm_id)
        else:
            velocity = data.get("velocity", 1.0)
            accel = data.get("accel", 1.0)
            decel = data.get("decel", 1.0)
            controller.set_speed(velocity, accel, decel, arm_id)
        
        sio.emit('speed:set', {"success": True, "params": controller.get_speed_params()}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"设置速度失败: {str(e)}"}, to=sid)


# ---------- 自由拖动 ----------

@sio.event
def freedrive_enable(sid, data):
    """启用自由拖动"""
    arm_id = data.get("arm_id")
    
    try:
        controller.enable_freedrive(arm_id)
        sio.emit('freedrive:enabled', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"启用自由拖动失败: {str(e)}"}, to=sid)


@sio.event
def freedrive_disable(sid, data):
    """退出自由拖动"""
    arm_id = data.get("arm_id")
    
    try:
        controller.disable_freedrive(arm_id)
        sio.emit('freedrive:disabled', {"arm_id": arm_id}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"退出自由拖动失败: {str(e)}"}, to=sid)


# ---------- 零点标定 ----------

@sio.event
def zero_calibrate(sid, data=None):
    """标定零点"""
    try:
        result = controller.calibrate_zero()
        sio.emit('zero:calibrated', {"result": result}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"零点标定失败: {str(e)}"}, to=sid)


@sio.event
def zero_load(sid, data=None):
    """查看零点偏移"""
    try:
        offsets = controller.get_zero_offsets()
        sio.emit('zero:loaded', {"offsets": offsets}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"获取零点失败: {str(e)}"}, to=sid)


# ---------- 示教功能 ----------

@sio.event
def teaching_record(sid, data):
    """
    记录当前位置
    
    data: {"name": "point_1", "arm_id": null}
    """
    name = data.get("name")
    arm_id = data.get("arm_id")
    
    try:
        point = controller.record_point(name, arm_id)
        sio.emit('teaching:recorded', {"point": point}, to=sid)
    except Exception as e:
        sio.emit('error', {"message": f"记录位置失败: {str(e)}"}, to=sid)


# ---------- 轨迹播放 ----------

@sio.event
def trajectory_play(sid, data):
    """
    播放轨迹
    
    data: {"filename": "preset_a.json", "sync": true}
    """
    filename = data.get("filename")
    sync = data.get("sync", True)
    
    if not filename:
        sio.emit('error', {"message": "缺少轨迹文件名"}, to=sid)
        return
    
    try:
        controller.play_trajectory(filename, sync)
        sio.emit('trajectory:started', {"filename": filename}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"播放轨迹失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_pause(sid, data=None):
    """暂停播放"""
    try:
        controller.pause_playback()
        sio.emit('trajectory:paused', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"暂停失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_resume(sid, data=None):
    """恢复播放"""
    try:
        controller.resume_playback()
        sio.emit('trajectory:resumed', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"恢复失败: {str(e)}"}, to=sid)


@sio.event
def trajectory_stop(sid, data=None):
    """停止播放"""
    try:
        controller.stop_playback()
        sio.emit('trajectory:stopped', {}, to=sid)
        _broadcast_state()
    except Exception as e:
        sio.emit('error', {"message": f"停止失败: {str(e)}"}, to=sid)


# ---------- 状态请求 ----------

@sio.event
def state_request(sid, data=None):
    """请求当前状态"""
    sio.emit('state:update', controller.get_state(), to=sid)


# ==================== 状态广播 ====================

def _broadcast_state():
    """广播状态给所有客户端"""
    try:
        sio.emit('state:update', controller.get_state())
    except Exception as e:
        print(f"[App] 广播状态失败: {e}")


def _state_update_loop():
    """状态更新循环"""
    global _state_update_running
    _state_update_running = True
    
    while _state_update_running:
        try:
            # 读取位置
            controller.read_positions()
            eventlet.sleep(0.05)  # 50ms
            
            # 广播状态
            _broadcast_state()
            
            # 等待间隔
            eventlet.sleep(STATE_UPDATE_INTERVAL / 1000.0)
        except Exception as e:
            print(f"[App] 状态更新出错: {e}")
            eventlet.sleep(1)


# ==================== 回调设置 ====================

def _on_error(message):
    """错误回调"""
    sio.emit('error', {"message": message})


def _on_playback_progress(info):
    """播放进度回调"""
    sio.emit('trajectory:progress', info)


# 设置控制器回调
controller.set_callbacks(
    on_error=_on_error,
    on_playback_progress=_on_playback_progress
)


# ==================== 主入口 ====================

def main():
    """主函数"""
    print("=" * 50)
    print("  机械臂演出控制系统")
    print("=" * 50)
    print(f"  访问地址: http://{HOST}:{PORT}")
    print(f"  调试模式: {DEBUG}")
    print("=" * 50)
    print()
    
    # 确保轨迹目录存在
    TRAJECTORIES_DIR.mkdir(parents=True, exist_ok=True)
    
    # 启动状态更新线程
    eventlet.spawn(_state_update_loop)
    
    # 启动服务
    eventlet.wsgi.server(
        eventlet.listen((HOST, PORT)),
        flask_app,
        log_output=DEBUG
    )


if __name__ == '__main__':
    main()
