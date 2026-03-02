#!/bin/bash
# CAN 接口完全重置脚本

echo "正在重置 CAN 接口..."
echo ""

# 1. 杀死所有可能占用 CAN 的进程
echo "[1/4] 停止占用 CAN 的进程..."
sudo pkill -9 candump 2>/dev/null
sudo pkill -9 cansniffer 2>/dev/null
sudo pkill -9 cansend 2>/dev/null
echo "  ✓ 已清理进程"

# 2. 关闭 CAN 接口
echo "[2/4] 关闭 CAN 接口..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null
sleep 1
echo "  ✓ 已关闭"

# 3. 清理队列并重新配置
echo "[3/4] 重新配置 CAN 接口..."
# 设置更大的发送队列（默认10，增加到100）
sudo ip link set can0 type can bitrate 1000000 txqueuelen 100 2>/dev/null
sudo ip link set can1 type can bitrate 1000000 txqueuelen 100 2>/dev/null

# 启动
sudo ip link set can0 up 2>/dev/null
sudo ip link set can1 up 2>/dev/null
sleep 1

# 4. 验证状态
echo "[4/4] 验证状态..."
CAN0_STATE=$(ip link show can0 2>/dev/null | grep -oP 'state \K\w+' || echo "ERROR")
CAN1_STATE=$(ip link show can1 2>/dev/null | grep -oP 'state \K\w+' || echo "ERROR")

if [ "$CAN0_STATE" = "UP" ]; then
    echo "  ✓ can0: $CAN0_STATE"
else
    echo "  ✗ can0: $CAN0_STATE"
fi

if [ "$CAN1_STATE" = "UP" ]; then
    echo "  ✓ can1: $CAN1_STATE"
else
    echo "  ✗ can1: $CAN1_STATE"
fi

echo ""
echo "CAN 接口重置完成！"
echo ""
echo "现在可以重新启动 Web 服务："
echo "  ./start.sh"
