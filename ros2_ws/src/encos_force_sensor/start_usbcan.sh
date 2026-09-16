#!/usr/bin/env bash
#
# 启动达妙 DM-USB2FDCAN（gs_usb 类 USB 转 CAN-FD 模块）并配置 CAN-FD 接口。
#
# 用法：
#   sudo ./start_usbcan.sh                    # 默认 1M 仲裁域 / 5M 数据域
#   sudo ./start_usbcan.sh 1000000 5000000    # 自定义波特率
#
# 前置条件（仅需一次）：gs_usb.ko 已编译安装到 /lib/modules/$(uname -r)/extra/
# 并执行过 depmod。详见同目录 docs/USB_CAN启动说明.md。

set -euo pipefail

ARB_BAUD="${1:-1000000}"     # 仲裁域波特率 (bps)
DATA_BAUD="${2:-5000000}"    # 数据域波特率 (bps)

# 1) 需要 root 权限
if [ "${EUID}" -ne 0 ]; then
    echo "错误：请用 sudo 运行（操作 CAN 接口需要 root 权限）" >&2
    echo "    sudo $0" >&2
    exit 1
fi

# 2) 加载 gs_usb 内核模块
if grep -q '^gs_usb ' /proc/modules; then
    echo "[1/3] gs_usb 内核模块已加载"
else
    echo "[1/3] 加载 gs_usb 内核模块..."
    if ! modprobe gs_usb; then
        echo "错误：无法加载 gs_usb 模块。" >&2
        echo "      请先编译安装 gs_usb.ko（见 docs/USB_CAN启动说明.md）" >&2
        exit 1
    fi
fi

# 3) 识别 gs_usb 对应的 CAN 接口（自动区分板载 mttcan 与 USB gs_usb）
echo "[2/3] 查找 gs_usb 对应的 CAN 接口..."
GS_IFS=()
for dev in /sys/class/net/can*; do
    [ -e "$dev" ] || continue
    name="$(basename "$dev")"
    drv="$(basename "$(readlink -f "$dev/device/driver" 2>/dev/null)" 2>/dev/null)"
    if [ "$drv" = "gs_usb" ]; then
        GS_IFS+=("$name")
    fi
done

if [ "${#GS_IFS[@]}" -eq 0 ]; then
    echo "错误：未找到 gs_usb 接口，请确认 USB 转 CAN-FD 模块已插入。" >&2
    exit 1
fi
echo "    找到接口：${GS_IFS[*]}"

# 4) 配置 CAN-FD
echo "[3/3] 配置 CAN-FD（仲裁域 ${ARB_BAUD} bps / 数据域 ${DATA_BAUD} bps）..."
for dev in "${GS_IFS[@]}"; do
    ip link set "$dev" down 2>/dev/null || true
    ip link set "$dev" up type can bitrate "${ARB_BAUD}" dbitrate "${DATA_BAUD}" fd on
    echo "    ${dev} 已配置"
done

echo ""
echo "完成。接口状态如下："
for dev in "${GS_IFS[@]}"; do
    ip -details link show "$dev" 2>/dev/null | grep -E "^[0-9]+:|state |bitrate|dbitrate"
    echo ""
done
echo "提示：可用 'candump <接口名>' 验证是否收到传感器帧。"
