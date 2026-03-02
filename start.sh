#!/bin/bash
# ============================================
#  机械臂演出控制系统 - Linux启动脚本
# ============================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "============================================"
echo "  机械臂演出控制系统"
echo "============================================"
echo ""

# 检查Python
if ! command -v python3 &> /dev/null; then
    echo "[错误] 未找到 python3"
    exit 1
fi

# 检查依赖
echo "[1/3] 检查依赖..."
python3 -c "import flask; import socketio; import eventlet" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[警告] 缺少依赖，正在安装..."
    pip install -r requirements.txt
fi

# 检查CAN接口
echo "[2/3] 检查CAN接口..."

CAN0_OK=false
CAN1_OK=false

if ip link show can0 &>/dev/null; then
    if ip link show can0 | grep -q "state UP"; then
        echo "  ✓ can0 已启动"
        CAN0_OK=true
    else
        echo "  ⚠ can0 未启动"
    fi
else
    echo "  ⚠ 未检测到 can0 接口"
fi

if ip link show can1 &>/dev/null; then
    if ip link show can1 | grep -q "state UP"; then
        echo "  ✓ can1 已启动"
        CAN1_OK=true
    else
        echo "  ⚠ can1 未启动"
    fi
else
    echo "  ⚠ 未检测到 can1 接口"
fi

# 如果CAN接口未启动，提示用户
if [ "$CAN0_OK" = false ] || [ "$CAN1_OK" = false ]; then
    echo ""
    echo "  ⚠️  CAN 接口未完全启动，机械臂将无法通讯！"
    echo ""
    echo "  快速修复："
    echo "    ./setup_can.sh"
    echo ""
    echo "  或手动执行："
    if [ "$CAN0_OK" = false ]; then
        echo "    sudo ip link set can0 up type can bitrate 1000000"
    fi
    if [ "$CAN1_OK" = false ]; then
        echo "    sudo ip link set can1 up type can bitrate 1000000"
    fi
    echo ""
    read -p "  是否继续启动服务？(y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "  已取消启动"
        exit 1
    fi
fi

# 启动服务
echo "[3/3] 启动Web服务..."
echo ""
echo "  访问地址: http://127.0.0.1:5000"
echo "  按 Ctrl+C 停止服务"
echo ""

# 后台启动浏览器 (延迟2秒)
(sleep 2 && xdg-open http://127.0.0.1:5000 2>/dev/null || open http://127.0.0.1:5000 2>/dev/null || echo "请手动打开浏览器访问 http://127.0.0.1:5000") &

# 启动Flask应用
python3 backend/app.py
