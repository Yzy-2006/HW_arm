#!/usr/bin/env python3
"""List RealSense devices with the fields needed by recording.yaml."""

import pyrealsense2 as rs


def main() -> None:
    devices = list(rs.context().query_devices())
    print(f"发现 {len(devices)} 台 RealSense 相机")
    for index, device in enumerate(devices, start=1):
        print(
            f"{index}. 名称: {device.get_info(rs.camera_info.name)}\n"
            f"   序列号: {device.get_info(rs.camera_info.serial_number)}"
        )


if __name__ == "__main__":
    main()
