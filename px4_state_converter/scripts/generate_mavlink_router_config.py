#!/usr/bin/env python3

import argparse
import pathlib
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a minimal mavlink-routerd config for PX4 <-> QGC WiFi relay."
    )
    parser.add_argument("--link-mode", choices=("serial", "udp"), required=True)
    parser.add_argument("--template", required=True, help="Template config file (.in)")
    parser.add_argument("--output", required=True, help="Output config path")
    parser.add_argument("--serial-device", default="", help="PX4 serial device path")
    parser.add_argument("--baudrate", type=int, default=921600, help="PX4 serial baudrate")
    parser.add_argument("--px4-udp-bind-port", type=int, default=14540, help="PX4 UDP bind port")
    parser.add_argument("--wifi-broadcast-ip", default="192.168.1.255", help="QGC WiFi broadcast IP")
    parser.add_argument("--qgc-udp-port", type=int, default=14550, help="QGC UDP listening port")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.link_mode == "serial" and not args.serial_device:
        raise ValueError("--serial-device is required when --link-mode=serial")

    if args.baudrate <= 0:
        raise ValueError("--baudrate must be positive")

    if args.px4_udp_bind_port <= 0 or args.px4_udp_bind_port > 65535:
        raise ValueError("--px4-udp-bind-port must be in 1..65535")

    if args.qgc_udp_port <= 0 or args.qgc_udp_port > 65535:
        raise ValueError("--qgc-udp-port must be in 1..65535")


def render_template(template_path: pathlib.Path, replacements: dict[str, str]) -> str:
    content = template_path.read_text(encoding="utf-8")
    for key, value in replacements.items():
        content = content.replace(f"@{key}@", value)
    return content


def main() -> int:
    args = parse_args()

    try:
        validate_args(args)
    except ValueError as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    template_path = pathlib.Path(args.template)
    output_path = pathlib.Path(args.output)

    if not template_path.is_file():
        print(f"Template not found: {template_path}", file=sys.stderr)
        return 2

    replacements = {
        "SERIAL_DEVICE": args.serial_device,
        "BAUDRATE": str(args.baudrate),
        "PX4_UDP_BIND_PORT": str(args.px4_udp_bind_port),
        "WIFI_BROADCAST_IP": args.wifi_broadcast_ip,
        "QGC_UDP_PORT": str(args.qgc_udp_port),
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_template(template_path, replacements), encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
