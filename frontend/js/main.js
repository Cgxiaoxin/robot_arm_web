/**
 * 机械臂演出控制系统 - 前端逻辑
 */

// ==================== Socket.IO 连接 ====================

const socket = io();

// 电机ID配置
const LEFT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57];
const RIGHT_MOTOR_IDS = [61, 62, 63, 64, 65, 66, 67];
const RIGHT_INVERT_MOTORS = new Set([61, 66]);

// 全局状态
let appState = {
    connected: false,
    arms: {
        left: { connected: false, initialized: false, motors: {} },
        right: { connected: false, initialized: false, motors: {} }
    },
    target: 'both',
    playback: {
        state: 'idle',
        progress: 0,
        trajectory_name: ''
    },
    trajectories: []
};

// ==================== Socket.IO 事件处理 ====================

socket.on('connect', () => {
    console.log('[Socket.IO] 已连接到服务器');
    updateConnectionStatus(true);
    addLog('已连接到服务器', 'success');
    loadTrajectories();
    // 连接成功后同步零点显示与速度滑杆
    loadZeroOffset();
    initSpeedSlider();
});

socket.on('disconnect', () => {
    console.log('[Socket.IO] 与服务器断开连接');
    updateConnectionStatus(false);
    addLog('与服务器断开连接', 'error');
});

socket.on('state:update', (state) => {
    appState = { ...appState, ...state };
    updateUI();
});

socket.on('arm:connected', (data) => {
    if (data.success) {
        addLog('机械臂连接成功', 'success');
    } else {
        addLog('机械臂连接失败', 'error');
    }
});

socket.on('arm:disconnected', (data) => {
    addLog(`${data.arm_id} 臂已断开`, 'info');
});

socket.on('arm:initialized', (data) => {
    const results = Object.entries(data.results)
        .map(([k, v]) => `${k}: ${v ? '成功' : '失败'}`)
        .join(', ');
    addLog(`初始化完成 - ${results}`, 'success');
});

socket.on('arm:deactivated', (data) => {
    addLog(`${data.arm_id} 已去使能`, 'info');
});

socket.on('arm:emergency_stopped', () => {
    addLog('!!! 紧急停止已执行 !!!', 'warning');
});

socket.on('trajectory:started', (data) => {
    addLog(`开始播放轨迹: ${data.filename}`, 'info');
});

socket.on('trajectory:progress', (data) => {
    updatePlaybackProgress(data);
});

socket.on('trajectory:paused', () => {
    addLog('轨迹已暂停', 'info');
});

socket.on('trajectory:resumed', () => {
    addLog('轨迹已恢复', 'info');
});

socket.on('trajectory:stopped', () => {
    addLog('轨迹已停止', 'info');
    updatePlaybackProgress({ progress: 0, current: 0, total: 0 });
});

socket.on('error', (data) => {
    addLog(`错误: ${data.message}`, 'error');
    showNotification(data.message, 'error');
});

socket.on('zero:calibrated', (data) => {
    const result = data?.result || {};
    const parts = [];
    let hasError = false;
    for (const [arm, status] of Object.entries(result)) {
        if (typeof status === 'string' && status.startsWith('error')) hasError = true;
        parts.push(`${arm}: ${status}`);
    }
    const msg = parts.length ? `零点标定结果：${parts.join(', ')}` : '零点标定完成';
    addLog(msg, hasError ? 'error' : 'success');
    showNotification(hasError ? msg : '零点标定成功', hasError ? 'error' : 'success');
    updateZeroStatus(hasError ? msg : '标定成功');
    // 标定后自动刷新显示
    loadZeroOffset();
});

socket.on('zero:loaded', (data) => {
    if (data.offsets) {
        renderZeroOffsets(data.offsets);
        updateZeroLastUpdated();
        updateZeroStatus('已更新');
        addLog('已加载零点偏移并更新到界面', 'info');
    }
});

socket.on('speed:set', (data) => {
    if (data?.success) {
        const p = data?.params;
        if (p) {
            syncSpeedUI(p.velocity, p.accel, p.decel);
        }
        showNotification('速度设置成功', 'success');
        addLog('速度设置成功', 'success');
    }
});

socket.on('target:changed', (data) => {
    appState.target = data.target;
    updateTargetUI();
    addLog(`控制目标已切换: ${data.target}`, 'info');
});

// ==================== UI 更新函数 ====================

function updateConnectionStatus(connected) {
    const dot = document.querySelector('.status-dot');
    const text = document.querySelector('.status-text');
    
    if (connected) {
        dot.classList.add('connected');
        dot.classList.remove('error');
        text.textContent = '服务器已连接';
    } else {
        dot.classList.remove('connected');
        dot.classList.add('error');
        text.textContent = '服务器未连接';
    }
}

function updateUI() {
    updateArmsStatus();
    updateMonitor();
    updatePlaybackUI();
    updateTargetUI();
}

function updateArmsStatus() {
    // 左臂状态
    const leftStatus = document.getElementById('left-arm-status');
    const leftState = appState.arms?.left;
    if (leftStatus && leftState) {
        leftStatus.className = 'arm-status';
        if (leftState.initialized) {
            leftStatus.classList.add('initialized');
            leftStatus.textContent = '已初始化';
        } else if (leftState.connected) {
            leftStatus.classList.add('connected');
            leftStatus.textContent = '已连接';
        } else {
            leftStatus.textContent = '未连接';
        }
    }
    
    // 右臂状态
    const rightStatus = document.getElementById('right-arm-status');
    const rightState = appState.arms?.right;
    if (rightStatus && rightState) {
        rightStatus.className = 'arm-status';
        if (rightState.initialized) {
            rightStatus.classList.add('initialized');
            rightStatus.textContent = '已初始化';
        } else if (rightState.connected) {
            rightStatus.classList.add('connected');
            rightStatus.textContent = '已连接';
        } else {
            rightStatus.textContent = '未连接';
        }
    }
    
    // 更新header状态
    const headerDot = document.querySelector('.status-dot');
    const anyConnected = leftState?.connected || rightState?.connected;
    if (anyConnected) {
        headerDot.classList.add('connected');
    } else {
        headerDot.classList.remove('connected');
    }
}

function updateMonitor() {
    updateArmMonitor('left');
    updateArmMonitor('right');
}

function updateArmMonitor(armId) {
    const tbody = document.getElementById(`${armId}-motors-tbody`);
    if (!tbody) return;
    
    const armState = appState.arms?.[armId];
    if (!armState || !armState.motors) {
        tbody.innerHTML = '<tr><td colspan="4" class="text-center">未连接</td></tr>';
        return;
    }
    
    const motorIds = armId === 'left' ? LEFT_MOTOR_IDS : RIGHT_MOTOR_IDS;
    let html = '';
    
    for (const mid of motorIds) {
        const motor = armState.motors[mid] || {};
        const position = motor.position !== undefined ? motor.position.toFixed(4) : '-';
        const degrees = motor.position !== undefined ? (motor.position * 180 / Math.PI).toFixed(1) : '-';
        const enabled = motor.enabled ? '使能' : '关闭';
        const enabledClass = motor.enabled ? 'enabled' : 'disabled';
        
        html += `
            <tr>
                <td>${mid}</td>
                <td class="position">${position}</td>
                <td>${degrees}°</td>
                <td class="${enabledClass}">${enabled}</td>
            </tr>
        `;
    }
    
    tbody.innerHTML = html;
}

function updatePlaybackUI() {
    const playback = appState.playback || {};
    const progressFill = document.querySelector('.progress-fill');
    const progressText = document.querySelector('.progress-text');
    
    if (progressFill) {
        progressFill.style.width = `${playback.progress || 0}%`;
    }
    
    if (progressText) {
        if (playback.state === 'playing') {
            progressText.textContent = `${playback.current_point || 0}/${playback.total_points || 0} - ${Math.round(playback.progress || 0)}%`;
        } else if (playback.state === 'paused') {
            progressText.textContent = '已暂停';
        } else {
            progressText.textContent = '就绪';
        }
    }
    
    // 更新按钮状态
    const playBtn = document.getElementById('btn-play');
    const pauseBtn = document.getElementById('btn-pause');
    const stopBtn = document.getElementById('btn-stop');
    
    if (playBtn) playBtn.disabled = playback.state === 'playing';
    if (pauseBtn) pauseBtn.disabled = playback.state !== 'playing';
    if (stopBtn) stopBtn.disabled = playback.state === 'idle';
}

function updatePlaybackProgress(data) {
    appState.playback = {
        ...appState.playback,
        progress: data.progress,
        current_point: data.current,
        total_points: data.total
    };
    updatePlaybackUI();
}

function updateTargetUI() {
    const options = document.querySelectorAll('.target-option');
    options.forEach(opt => {
        const value = opt.dataset.target;
        if (value === appState.target) {
            opt.classList.add('active');
        } else {
            opt.classList.remove('active');
        }
    });
}

// ==================== 操作函数 ====================

function connectArms() {
    const leftChannel = document.getElementById('left-channel')?.value || 'can0';
    const rightChannel = document.getElementById('right-channel')?.value || 'can1';
    
    socket.emit('arm_connect', {
        arm_id: 'both',
        left_channel: leftChannel,
        right_channel: rightChannel
    });
    addLog('正在连接机械臂...', 'info');
}

function disconnectArms() {
    socket.emit('arm_disconnect', { arm_id: 'both' });
}

function initArms() {
    const mode = parseInt(document.getElementById('init-mode')?.value || '6');
    socket.emit('arm_init', { mode: mode });
    addLog('正在启动使能机械臂...', 'info');
}

function deactivateArms() {
    socket.emit('arm_deactivate', {});
    addLog('正在去使能...', 'info');
}

function disableArms() {
    socket.emit('arm_disable', {});
    addLog('正在关闭电机...', 'info');
}

function emergencyStop() {
    socket.emit('emergency_stop', {});
    addLog('!!! 发送紧急停止命令 !!!', 'warning');
}

function goToZero() {
    socket.emit('go_to_zero', {});
    addLog('正在回零...', 'info');
}

function setTarget(target) {
    socket.emit('set_target', { target: target });
}

function loadTrajectories() {
    fetch('/api/trajectories')
        .then(res => res.json())
        .then(data => {
            appState.trajectories = data.trajectories || [];
            updateTrajectorySelect();
        })
        .catch(err => {
            console.error('加载轨迹列表失败:', err);
        });
}

function updateTrajectorySelect() {
    const select = document.getElementById('trajectory-select');
    if (!select) return;
    
    select.innerHTML = '<option value="">选择轨迹文件...</option>';
    for (const file of appState.trajectories) {
        select.innerHTML += `<option value="${file}">${file}</option>`;
    }
}

function playTrajectory() {
    const filename = document.getElementById('trajectory-select')?.value;
    if (!filename) {
        showNotification('请先选择轨迹文件', 'warning');
        return;
    }
    socket.emit('trajectory_play', { filename: filename, sync: true });
}

function pauseTrajectory() {
    socket.emit('trajectory_pause', {});
}

function resumeTrajectory() {
    if (appState.playback?.state === 'paused') {
        socket.emit('trajectory_resume', {});
    } else {
        playTrajectory();
    }
}

function stopTrajectory() {
    socket.emit('trajectory_stop', {});
}

function playPreset(presetId) {
    const filename = `preset_${presetId.toLowerCase()}.json`;
    socket.emit('trajectory_play', { filename: filename, sync: true });
    addLog(`执行预设动作: ${presetId}`, 'info');
    
    // 高亮当前按钮
    document.querySelectorAll('.preset-btn').forEach(btn => btn.classList.remove('active'));
    document.querySelector(`[data-preset="${presetId}"]`)?.classList.add('active');
}

function setSpeed(preset) {
    socket.emit('set_speed', { preset: preset });
    addLog(`设置速度: ${preset}`, 'info');
}

function setCustomSpeed() {
    const velocity = parseFloat(document.getElementById('speed-velocity')?.value || '1.0');
    const accel = parseFloat(document.getElementById('speed-accel')?.value || '1.0');
    const decel = parseFloat(document.getElementById('speed-decel')?.value || '1.0');
    
    socket.emit('set_speed', { velocity, accel, decel });
    addLog(`设置自定义速度: v=${velocity}, a=${accel}, d=${decel}`, 'info');
}

function enableFreedrive() {
    socket.emit('freedrive_enable', {});
    addLog('启用自由拖动模式', 'info');
}

function disableFreedrive() {
    socket.emit('freedrive_disable', {});
    addLog('退出自由拖动模式', 'info');
}

function jogMotor(armId, motorId, direction) {
    const step = parseFloat(document.getElementById('jog-step')?.value || '0.1');
    const currentPos = appState.arms?.[armId]?.motors?.[motorId]?.position || 0;
    let dir = direction;
    if (armId === 'right' && RIGHT_INVERT_MOTORS.has(motorId)) {
        dir = -dir;
    }
    const newPos = currentPos + (dir * step);
    
    socket.emit('manual_set_position', {
        arm_id: armId,
        motor_id: motorId,
        position: newPos
    });
}

function zeroMotor(armId, motorId) {
    socket.emit('manual_set_position_offset', {
        arm_id: armId,
        motor_id: motorId,
        position: 0
    });
}

function calibrateZero() {
    socket.emit('zero_calibrate', {});
    addLog('正在标定零点...', 'info');
    updateZeroStatus('标定中...');
}

function loadZeroOffset() {
    socket.emit('zero_load', {});
    addLog('正在查看零点偏移...', 'info');
}

function updateZeroStatus(text) {
    const el = document.getElementById('zero-calib-status');
    if (!el) return;
    el.textContent = text || '';
}

function updateZeroLastUpdated() {
    const el = document.getElementById('zero-last-updated');
    if (!el) return;
    el.textContent = new Date().toLocaleString();
}

function renderZeroOffsets(offsetsByArm) {
    const leftEl = document.getElementById('zero-left-list');
    const rightEl = document.getElementById('zero-right-list');
    if (!leftEl || !rightEl) return;

    const fmt = (obj) => {
        if (!obj || Object.keys(obj).length === 0) return '无数据';
        const lines = Object.entries(obj)
            .sort((a, b) => Number(a[0]) - Number(b[0]))
            .map(([mid, off]) => `电机 ${mid}: ${Number(off).toFixed(4)} rad`);
        return lines.join('\n');
    };

    leftEl.textContent = fmt(offsetsByArm.left);
    rightEl.textContent = fmt(offsetsByArm.right);
}

// ==================== 速度滑杆 ====================

let _speedSliderDebounce = null;

function initSpeedSlider() {
    const slider = document.getElementById('speed-slider');
    const velInput = document.getElementById('speed-velocity');
    if (!slider || !velInput) return;

    // 暂时使用页面已有上限（未知电机上限时不强行猜）
    const max = parseFloat(slider.max || velInput.max || '2.0');
    slider.min = '0';
    slider.max = String(isFinite(max) ? max : 2.0);
    slider.step = '0.01';

    const maxEl = document.getElementById('speed-max');
    const minEl = document.getElementById('speed-min');
    if (maxEl) maxEl.textContent = slider.max;
    if (minEl) minEl.textContent = slider.min;

    // 初始同步
    const initV = clamp(parseFloat(velInput.value || '1.0'), parseFloat(slider.min), parseFloat(slider.max));
    slider.value = String(initV);
    updateSpeedSliderValue(initV);

    slider.addEventListener('input', () => {
        const v = clamp(parseFloat(slider.value), parseFloat(slider.min), parseFloat(slider.max));
        velInput.value = v.toFixed(2);
        updateSpeedSliderValue(v);

        // 200ms 去抖发送，避免拖动时刷屏
        if (_speedSliderDebounce) clearTimeout(_speedSliderDebounce);
        _speedSliderDebounce = setTimeout(() => {
            setCustomSpeed();
        }, 200);
    });

    velInput.addEventListener('change', () => {
        const v = clamp(parseFloat(velInput.value || '0'), parseFloat(slider.min), parseFloat(slider.max));
        velInput.value = v.toFixed(2);
        slider.value = String(v);
        updateSpeedSliderValue(v);
    });
}

function updateSpeedSliderValue(v) {
    const el = document.getElementById('speed-slider-value');
    const cur = document.getElementById('speed-current');
    if (el) el.textContent = Number(v).toFixed(2);
    if (cur) cur.textContent = Number(v).toFixed(2);
}

function syncSpeedUI(velocity, accel, decel) {
    const v = Number(velocity);
    const a = Number(accel);
    const d = Number(decel);
    const velInput = document.getElementById('speed-velocity');
    const accelInput = document.getElementById('speed-accel');
    const decelInput = document.getElementById('speed-decel');
    const slider = document.getElementById('speed-slider');

    if (velInput && isFinite(v)) velInput.value = v.toFixed(2);
    if (accelInput && isFinite(a)) accelInput.value = a.toFixed(2);
    if (decelInput && isFinite(d)) decelInput.value = d.toFixed(2);

    if (slider && isFinite(v)) {
        const min = parseFloat(slider.min || '0');
        const max = parseFloat(slider.max || '2');
        const clamped = clamp(v, min, max);
        slider.value = String(clamped);
        updateSpeedSliderValue(clamped);
    }
}

function clamp(x, min, max) {
    if (!isFinite(x)) return min;
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

function recordPoint() {
    const name = document.getElementById('point-name')?.value || '';
    socket.emit('teaching_record', { name: name });
    addLog(`记录位置点: ${name || '(unnamed)'}`, 'info');
}

// ==================== Tab 切换 ====================

function switchTab(tabId) {
    // 切换按钮状态
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.tab === tabId);
    });
    
    // 切换内容
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.toggle('active', content.id === `tab-${tabId}`);
    });
}

// ==================== 日志 ====================

function addLog(message, type = 'info') {
    const container = document.getElementById('log-container');
    if (!container) return;
    
    const time = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.innerHTML = `<span class="log-time">[${time}]</span> ${message}`;
    
    container.appendChild(entry);
    container.scrollTop = container.scrollHeight;
    
    // 限制日志条数
    while (container.children.length > 100) {
        container.removeChild(container.firstChild);
    }
}

function clearLog() {
    const container = document.getElementById('log-container');
    if (container) {
        container.innerHTML = '';
    }
}

// ==================== 通知 ====================

function showNotification(message, type = 'info') {
    // Toast 通知（可见）
    let container = document.getElementById('toast-container');
    if (!container) {
        container = document.createElement('div');
        container.id = 'toast-container';
        container.className = 'toast-container';
        document.body.appendChild(container);
    }

    const t = String(type || 'info').toLowerCase();
    const toast = document.createElement('div');
    toast.className = `toast toast-${t}`;

    const titleMap = {
        success: '成功',
        warning: '提示',
        error: '错误',
        info: '信息'
    };
    const title = titleMap[t] || '信息';

    toast.innerHTML = `
        <div class="toast-title">${title}</div>
        <div class="toast-msg">${escapeHtml(String(message || ''))}</div>
    `;
    container.appendChild(toast);

    setTimeout(() => {
        toast.remove();
    }, 2600);
}

function escapeHtml(str) {
    return str
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#039;');
}

// ==================== 初始化 ====================

document.addEventListener('DOMContentLoaded', () => {
    console.log('[App] 页面加载完成');
    
    // 绑定Tab切换
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', () => switchTab(btn.dataset.tab));
    });
    
    // 绑定目标选择
    document.querySelectorAll('.target-option').forEach(opt => {
        opt.addEventListener('click', () => setTarget(opt.dataset.target));
    });
    
    // 默认显示第一个Tab
    switchTab('manual');
    
    addLog('系统初始化完成', 'success');
    // 先把速度滑杆与输入框同步一次（连接后会再次初始化）
    initSpeedSlider();
});

// ==================== 导出全局函数 ====================

window.connectArms = connectArms;
window.disconnectArms = disconnectArms;
window.initArms = initArms;
window.deactivateArms = deactivateArms;
window.disableArms = disableArms;
window.emergencyStop = emergencyStop;
window.goToZero = goToZero;
window.setTarget = setTarget;
window.playTrajectory = playTrajectory;
window.pauseTrajectory = pauseTrajectory;
window.resumeTrajectory = resumeTrajectory;
window.stopTrajectory = stopTrajectory;
window.playPreset = playPreset;
window.setSpeed = setSpeed;
window.setCustomSpeed = setCustomSpeed;
window.enableFreedrive = enableFreedrive;
window.disableFreedrive = disableFreedrive;
window.jogMotor = jogMotor;
window.zeroMotor = zeroMotor;
window.calibrateZero = calibrateZero;
window.loadZeroOffset = loadZeroOffset;
window.recordPoint = recordPoint;
window.switchTab = switchTab;
window.clearLog = clearLog;
