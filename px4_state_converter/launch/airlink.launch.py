#!/usr/bin/env python3

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

def _as_positive_int(raw_value: str, field_name: str) -> int:
    try:
        value = int(raw_value)
    except ValueError as exc:
        raise RuntimeError(f"{field_name} must be an integer, got '{raw_value}'.") from exc

    if value <= 0 or value > 65535:
        raise RuntimeError(f"{field_name} must be in 1..65535, got '{raw_value}'.")
    return value


def _setup_airlink(context):
    mavlink_routerd = shutil.which("mavlink-routerd")
    if not mavlink_routerd:
        raise RuntimeError(
            "mavlink-routerd was not found in PATH. Please install mavlink-routerd on the onboard computer."
        )

    package_name = "px4_state_converter"
    package_prefix = get_package_prefix(package_name)
    package_share = get_package_share_directory(package_name)
    generator = os.path.join(
        package_prefix, "lib", package_name, "generate_mavlink_router_config.py"
    )

    if not os.path.isfile(generator):
        raise RuntimeError(f"Config generator not found: {generator}")

    link_mode = LaunchConfiguration("link_mode").perform(context).strip().lower()
    serial_device = LaunchConfiguration("serial_device").perform(context).strip()
    wifi_broadcast_ip = LaunchConfiguration("wifi_broadcast_ip").perform(context).strip()
    if link_mode not in {"serial", "udp"}:
        raise RuntimeError("link_mode must be either 'serial' or 'udp'.")

    baudrate = 921600
    px4_udp_bind_port = 14540
    qgc_udp_port = 14550
    log_output = "screen"

    if not wifi_broadcast_ip:
        raise RuntimeError("wifi_broadcast_ip must not be empty.")

    template_name = (
        "mavlink_router_serial.conf.in"
        if link_mode == "serial"
        else "mavlink_router_udp.conf.in"
    )
    template_path = os.path.join(package_share, "config", template_name)
    config_path = os.path.join(
        "/tmp", "px4_state_converter", "airlink", f"{link_mode}_mavlink_router.conf"
    )

    cmd = [
        generator,
        "--link-mode",
        link_mode,
        "--template",
        template_path,
        "--output",
        config_path,
        "--serial-device",
        serial_device,
        "--baudrate",
        str(baudrate),
        "--px4-udp-bind-port",
        str(px4_udp_bind_port),
        "--wifi-broadcast-ip",
        wifi_broadcast_ip,
        "--qgc-udp-port",
        str(qgc_udp_port),
    ]

    result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        stderr = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise RuntimeError(f"Failed to generate mavlink-router config: {stderr}")

    if not os.path.isfile(config_path):
        raise RuntimeError(f"mavlink-router config file does not exist: {config_path}")

    info_lines = [
        f"[AirLink] Starting mavlink-routerd with link_mode={link_mode}",
        f"[AirLink] Config file: {config_path}",
        f"[AirLink] WiFi broadcast target: {wifi_broadcast_ip}:{qgc_udp_port}",
    ]

    if link_mode == "serial":
        info_lines.append(
            f"[AirLink] PX4 serial endpoint: {serial_device} @ {baudrate}"
        )
    else:
        info_lines.append(
            f"[AirLink] PX4 UDP bind port: 0.0.0.0:{px4_udp_bind_port}"
        )

    return [LogInfo(msg=line) for line in info_lines] + [
        ExecuteProcess(
            cmd=[mavlink_routerd, "-c", config_path],
            output=log_output,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "link_mode",
                default_value="serial",
                description="PX4 uplink type: 'serial' or 'udp'.",
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value="/dev/ttyUSB0",
                description="PX4 MAVLink serial device when link_mode=serial.",
            ),
            DeclareLaunchArgument(
                "wifi_broadcast_ip",
                default_value="192.168.1.255",
                description="Broadcast address used to forward MAVLink telemetry to QGC over WiFi.",
            ),
            OpaqueFunction(function=_setup_airlink),
        ]
    )
