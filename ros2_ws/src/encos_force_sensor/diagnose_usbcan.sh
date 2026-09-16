#!/usr/bin/env bash
#
# USB-CAN 一键诊断脚本：加载 gs_usb 驱动 → 配置 CAN-FD → 抓帧显示各接口收到的 CAN ID，
# 并与 config/force_sensor.yaml 里的 device/can_id 对照，方便开机后快速核对接线与接口映射。
#
# 用法：
#   sudo ./diagnose_usbcan.sh                       # 默认 1M 仲裁域 / 5M 数据域，抓 3 秒
#   sudo ./diagnose_usbcan.sh 1000000 5000000 5     # 自定义仲裁波特率 / 数据波特率 / 抓帧秒数
#
# 输出说明：
#   - 每个 gs_usb 接口一行「CAN ID + 帧长 + 帧数」；帧长 [32] 表示 CAN-FD 32 字节帧（力传感器标准）。
#   - 末尾打印 yaml 里的 device/can_id 配置，两者并列即可直接对照。
#
# 前置条件（仅需一次）：gs_usb.ko 已编译安装到 /lib/modules/$(uname -r)/extra/
# 并执行过 depmod，详见同目录 docs/USB_CAN启动说明.md。

set -euo pipefail

ARB_BAUD="${1:-1000000}"     # 仲裁域波特率 (bps)
DATA_BAUD="${2:-5000000}"    # 数据域波特率 (bps)
SNIFF_SECS="${3:-3}"         # 每个接口抓帧时长（秒）

# 依赖检查
if ! command -v candump >/dev/null 2>&1; then
    echo "错误：缺少 candump，请先安装 can-utils：sudo apt install -y can-utils" >&2
    exit 1
fi

# 1) 需要 root 权限
if [ "${EUID}" -ne 0 ]; then
    echo "错误：请用 sudo 运行（操作 CAN 接口需要 root 权限）" >&2
    echo "    sudo $0" >&2
    exit 1
fi

# 2) 加载 gs_usb 内核模块
if grep -q '^gs_usb ' /proc/modules; then
    echo "[1/4] gs_usb 内核模块已加载"
else
    echo "[1/4] 加载 gs_usb 内核模块..."
    if ! modprobe gs_usb; then
        echo "错误：无法加载 gs_usb 模块。" >&2
        echo "      请先编译安装 gs_usb.ko（见 docs/USB_CAN启动说明.md）" >&2
        exit 1
    fi
fi

# 3) 识别 gs_usb 对应的 CAN 接口（自动区分板载 mttcan 与 USB gs_usb）
echo "[2/4] 查找 gs_usb 对应的 CAN 接口..."
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
    echo "      可用 lsusb 确认存在 1d50:606f 设备。" >&2
    exit 1
fi
echo "    找到接口：${GS_IFS[*]}"

# 4) 配置 CAN-FD
echo "[3/4] 配置 CAN-FD（仲裁域 ${ARB_BAUD} bps / 数据域 ${DATA_BAUD} bps）..."
for dev in "${GS_IFS[@]}"; do
    ip link set "$dev" down 2>/dev/null || true
    ip link set "$dev" up type can bitrate "${ARB_BAUD}" dbitrate "${DATA_BAUD}" fd on
done

# 5) 抓帧统计各接口收到的 CAN ID
echo "[4/4] 抓帧 ${SNIFF_SECS} 秒，统计各接口收到的 CAN ID..."
echo ""
for dev in "${GS_IFS[@]}"; do
    echo "───── ${dev} ─────"
    # candump 输出形如 "  can0  101  [32]  xx ..."；取 ID($2) 与帧长($3) 统计。
    # 命令替换内 || true 吸收 timeout 到点杀掉 candump 的非零退出码。
    out="$(timeout "${SNIFF_SECS}" candump "$dev" 2>/dev/null \
           | awk '{print $2, $3}' | sort | uniq -c | sort -rn || true)"
    if [ -z "$out" ]; then
        echo "  （未收到任何帧）→ 检查传感器供电、接线、终端电阻(120Ω)、波特率是否 ${ARB_BAUD}/${DATA_BAUD}"
    else
        echo "$out" | awk '{printf "  CAN ID 0x%-8s 帧长%-5s 帧数=%s\n", $2, $3, $1}'
    fi
    echo ""
done

# 6) 打印 yaml 里的 device/can_id 配置做对照
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"
YAML=""
for p in \
    "${WS_DIR}/install/encos_force_sensor/share/encos_force_sensor/config/force_sensor.yaml" \
    "${SCRIPT_DIR}/config/force_sensor.yaml"; do
    if [ -f "$p" ]; then YAML="$p"; break; fi
done

echo "───── 当前 yaml 配置对照 ─────"
if [ -z "$YAML" ]; then
    echo "  （未找到 config/force_sensor.yaml）"
else
    awk '
    {
      line = $0
      sub(/[ \t]*#.*/, "", line)          # 去掉行内注释
      gsub(/^[ \t]+|[ \t]+$/, "", line)   # 去首尾空白
      if (line ~ /^[A-Za-z_]+:$/) {        # 传感器名行（如 left:）
        gsub(/:$/, "", line)
        sensor = line
      } else if (line ~ /^device:/) {
        printf "  %-10s %s\n", sensor, line
      } else if (line ~ /^can_id:/) {
        printf "  %-10s %s\n", sensor, line
      }
    }' "$YAML"
    echo "  (来源: ${YAML})"
fi

echo ""
echo "提示：把上面「───── <接口> ─────」里实际收到的 CAN ID，对应填到 yaml 中该传感器的 device/can_id。"
echo "      若两个接口上的 ID 与 yaml 对不上（或相互对调），先确认接线，再改 config/force_sensor.yaml 并重新 colcon build。"
