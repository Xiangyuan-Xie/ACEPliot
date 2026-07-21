"""Nature-style figure defaults and publication export helpers."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile


COLORS = {
    "actual": "#0F4D92",
    "actual_light": "#3775BA",
    "reference": "#767676",
    "error_x": "#3775BA",
    "error_y": "#42949E",
    "error_z": "#9A4D8E",
    "error_norm": "#B64342",
    "neutral_light": "#CFCECE",
    "neutral_dark": "#4D4D4D",
    "start": "#2E9E44",
    "end": "#B64342",
}


def pyplot():
    """Return a headless pyplot module configured for publication figures."""
    cache_root = Path(tempfile.gettempdir()) / "acepliot-plot-cache"
    matplotlib_cache = cache_root / "matplotlib"
    xdg_cache = cache_root / "xdg"
    matplotlib_cache.mkdir(parents=True, exist_ok=True)
    xdg_cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLBACKEND", "Agg")
    os.environ.setdefault("MPLCONFIGDIR", str(matplotlib_cache))
    os.environ.setdefault("XDG_CACHE_HOME", str(xdg_cache))

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [
                "Arial",
                "Helvetica",
                "DejaVu Sans",
                "sans-serif",
            ],
            "font.size": 8.0,
            "axes.labelsize": 8.0,
            "axes.titlesize": 8.0,
            "axes.linewidth": 0.8,
            "axes.spines.right": False,
            "axes.spines.top": False,
            "xtick.labelsize": 7.0,
            "ytick.labelsize": 7.0,
            "xtick.major.width": 0.7,
            "ytick.major.width": 0.7,
            "legend.fontsize": 7.0,
            "legend.frameon": False,
            "lines.linewidth": 1.4,
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "savefig.facecolor": "white",
            "figure.facecolor": "white",
        }
    )
    return plt


def add_panel_label(axis, label: str, *, is_3d: bool = False) -> None:
    """Add a compact lowercase panel label at the upper-left edge."""
    if is_3d:
        axis.text2D(
            0.01,
            0.98,
            label,
            transform=axis.transAxes,
            ha="left",
            va="top",
            fontweight="bold",
            fontsize=8.0,
        )
        return
    axis.text(
        -0.08,
        1.02,
        label,
        transform=axis.transAxes,
        ha="left",
        va="bottom",
        fontweight="bold",
        fontsize=8.0,
    )


def save_figure_bundle(figure, output_dir: Path, stem: str) -> list[Path]:
    """Save a PNG preview and editable SVG/PDF publication files."""
    if not stem or Path(stem).name != stem or Path(stem).suffix:
        raise ValueError("figure stem must be a non-empty filename without a suffix")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = [output_dir / f"{stem}.{suffix}" for suffix in ("png", "svg", "pdf")]
    figure.savefig(paths[0], dpi=300, bbox_inches="tight", facecolor="white")
    figure.savefig(paths[1], bbox_inches="tight", facecolor="white")
    figure.savefig(paths[2], bbox_inches="tight", facecolor="white")
    return paths
