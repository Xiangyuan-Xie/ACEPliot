#!/usr/bin/env python3

import argparse
import pathlib
import sys

import yaml


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
DEFAULT_OUTPUT = pathlib.Path("/etc/mavlink-router/main.conf")


def _require_mapping(value, field_name: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping.")
    return value


def _as_positive_int(raw_value, field_name: str) -> int:
    try:
        value = int(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be an integer.") from exc

    if value <= 0:
        raise ValueError(f"{field_name} must be positive.")
    return value


def _as_port(raw_value, field_name: str) -> int:
    value = _as_positive_int(raw_value, field_name)
    if value > 65535:
        raise ValueError(f"{field_name} must be in 1..65535.")
    return value


def load_config(config_path: pathlib.Path) -> dict:
    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file)

    if not isinstance(loaded, dict):
        raise ValueError("Top-level YAML document must be a mapping.")
    return loaded


def validate_config(config: dict) -> dict:
    link_mode = str(config.get("link_mode", "")).strip().lower()
    if link_mode not in {"serial", "udp"}:
        raise ValueError("link_mode must be either 'serial' or 'udp'.")

    serial = _require_mapping(config.get("serial", {}), "serial")
    px4_udp = _require_mapping(config.get("px4_udp", {}), "px4_udp")
    gcs_udp = _require_mapping(config.get("gcs_udp", {}), "gcs_udp")

    serial_device = str(serial.get("device", "")).strip()
    if link_mode == "serial" and not serial_device:
        raise ValueError("serial.device is required when link_mode=serial.")

    serial_baudrate = _as_positive_int(serial.get("baudrate", 921600), "serial.baudrate")
    px4_udp_bind_address = str(px4_udp.get("bind_address", "0.0.0.0")).strip() or "0.0.0.0"
    px4_udp_bind_port = _as_port(px4_udp.get("bind_port", 14540), "px4_udp.bind_port")
    gcs_udp_bind_address = str(gcs_udp.get("bind_address", "0.0.0.0")).strip() or "0.0.0.0"
    gcs_udp_bind_port = _as_port(gcs_udp.get("bind_port", 14550), "gcs_udp.bind_port")

    return {
        "link_mode": link_mode,
        "serial_device": serial_device,
        "serial_baudrate": serial_baudrate,
        "px4_udp_bind_address": px4_udp_bind_address,
        "px4_udp_bind_port": px4_udp_bind_port,
        "gcs_udp_bind_address": gcs_udp_bind_address,
        "gcs_udp_bind_port": gcs_udp_bind_port,
    }


def render_template(template_path: pathlib.Path, replacements: dict[str, str]) -> str:
    content = template_path.read_text(encoding="utf-8")
    for key, value in replacements.items():
        content = content.replace(f"@{key}@", value)
    return content


def generate_config(config_path: pathlib.Path, output_path: pathlib.Path) -> pathlib.Path:
    config = validate_config(load_config(pathlib.Path(config_path)))
    template_name = (
        "mavlink_router_serial.conf.in"
        if config["link_mode"] == "serial"
        else "mavlink_router_udp.conf.in"
    )
    template_path = SCRIPT_DIR / "templates" / template_name
    if not template_path.is_file():
        raise FileNotFoundError(f"Template not found: {template_path}")

    replacements = {
        "SERIAL_DEVICE": config["serial_device"],
        "BAUDRATE": str(config["serial_baudrate"]),
        "PX4_UDP_BIND_ADDRESS": config["px4_udp_bind_address"],
        "PX4_UDP_BIND_PORT": str(config["px4_udp_bind_port"]),
        "GCS_BIND_ADDRESS": config["gcs_udp_bind_address"],
        "GCS_UDP_PORT": str(config["gcs_udp_bind_port"]),
    }

    output_path = pathlib.Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_template(template_path, replacements), encoding="utf-8")
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate /etc/mavlink-router/main.conf from a repository YAML scenario."
    )
    parser.add_argument("config_path", help="Path to a YAML scenario file under tools/airlink/configs")
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="Destination mavlink-router config path. Defaults to /etc/mavlink-router/main.conf.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        generated = generate_config(pathlib.Path(args.config_path), pathlib.Path(args.output))
    except (FileNotFoundError, ValueError, yaml.YAMLError) as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    print(generated)
    return 0


if __name__ == "__main__":
    sys.exit(main())
