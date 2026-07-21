"""Filesystem helpers for benchmark artifacts."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Iterable, Mapping, Sequence


def _find_source_package(package_name: str, anchors: Sequence[Path]) -> Path | None:
    seen: set[Path] = set()
    for anchor in anchors:
        for root in (anchor, *anchor.parents):
            candidate = root / "benchmarks" / package_name
            if candidate in seen:
                continue
            seen.add(candidate)
            if (candidate / "package.xml").is_file():
                return candidate
    return None


def _find_workspace_source_package(
    package_name: str, source_root: Path
) -> Path | None:
    anchors: list[Path] = []
    for entry in source_root.iterdir():
        try:
            resolved = entry.resolve(strict=True)
        except (FileNotFoundError, RuntimeError):
            continue
        if resolved.is_dir():
            anchors.append(resolved)

    source_package = _find_source_package(package_name, anchors)
    if source_package is not None:
        return source_package

    pattern = f"**/benchmarks/{package_name}/package.xml"
    for manifest in source_root.glob(pattern):
        return manifest.parent.resolve()
    return None


def find_package_directory(package_name: str) -> Path:
    """Prefer a benchmark's source directory and fall back to its install share."""
    if not package_name or not package_name.replace("_", "").isalnum():
        raise ValueError("package_name must be a non-empty ROS package name")

    source_package = _find_source_package(package_name, [Path.cwd().resolve()])
    if source_package is not None:
        return source_package

    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )

    prefix = Path(get_package_prefix(package_name)).resolve()
    source_package = _find_source_package(package_name, [prefix])
    if source_package is not None:
        return source_package

    install_roots = [root for root in (prefix, *prefix.parents) if root.name == "install"]
    for install_root in install_roots:
        source_root = install_root.parent / "src"
        if not source_root.is_dir():
            continue
        source_package = _find_workspace_source_package(package_name, source_root)
        if source_package is not None:
            return source_package

    return Path(get_package_share_directory(package_name)).resolve()


class ReportWriter:
    """Write one benchmark run to a deterministic artifact directory."""

    def __init__(
        self,
        benchmark_name: str,
        output_dir: str = "",
        package_name: str = "benchmark_reporting",
        package_directory: str | Path | None = None,
    ) -> None:
        if output_dir:
            requested = Path(output_dir).expanduser()
            if not requested.is_absolute():
                raise ValueError("output_dir must be empty or an absolute path")
            self.output_dir = requested
        else:
            package_root = (
                Path(package_directory).expanduser().resolve()
                if package_directory is not None
                else find_package_directory(package_name)
            )
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            self.output_dir = package_root / "logs" / timestamp
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def write_json(self, name: str, payload: Mapping[str, object]) -> Path:
        path = self.output_dir / name
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
        return path

    def write_csv(self, name: str, rows: Iterable[Mapping[str, object]]) -> Path:
        rows = list(rows)
        path = self.output_dir / name
        if not rows:
            path.write_text("", encoding="utf-8")
            return path
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        return path


def artifact_names(paths: Iterable[Path]) -> list[str]:
    """Return artifact basenames for the JSON summary."""
    return [path.name for path in paths]
