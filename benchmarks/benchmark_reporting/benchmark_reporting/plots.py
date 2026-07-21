"""Publication-quality benchmark plots built from aligned numerical data."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from benchmark_reporting.figure_style import (
    COLORS,
    add_panel_label,
    pyplot,
    save_figure_bundle,
)


AXIS_NAMES = ("East", "North", "Up")
AXIS_SYMBOLS = ("E", "N", "U")
ERROR_COLORS = (COLORS["error_x"], COLORS["error_y"], COLORS["error_z"])


def _as_xyz(name: str, values: Sequence[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(values, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != 3 or len(matrix) == 0:
        raise ValueError(f"{name} must be a non-empty Nx3 matrix")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must contain only finite values")
    return matrix


def _as_times(name: str, values: Sequence[float], expected: int) -> np.ndarray:
    times = np.asarray(values, dtype=float)
    if times.ndim != 1 or len(times) != expected:
        raise ValueError(f"{name} must contain one timestamp per sample")
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) < 0.0):
        raise ValueError(f"{name} must be finite and non-decreasing")
    return times


def _prepare_trajectory(
    actual: Sequence[Sequence[float]],
    reference: Sequence[Sequence[float]] | None = None,
    origin: Sequence[float] | None = None,
) -> tuple[np.ndarray, np.ndarray | None, bool]:
    actual_matrix = _as_xyz("actual trajectory", actual).copy()
    reference_matrix = None
    if reference is not None:
        reference_matrix = _as_xyz("reference trajectory", reference).copy()
    relative = origin is not None
    if relative:
        origin_vector = np.asarray(origin, dtype=float)
        if origin_vector.shape != (3,) or not np.all(np.isfinite(origin_vector)):
            raise ValueError("trajectory origin must contain three finite values")
        actual_matrix -= origin_vector
        if reference_matrix is not None:
            reference_matrix -= origin_vector
    return actual_matrix, reference_matrix, relative


def _cube_limits(*matrices: np.ndarray | None) -> tuple[tuple[float, float], ...]:
    points = [matrix for matrix in matrices if matrix is not None and len(matrix)]
    if not points:
        raise ValueError("at least one trajectory is required")
    combined = np.vstack(points)
    lower = np.min(combined, axis=0)
    upper = np.max(combined, axis=0)
    centers = 0.5 * (lower + upper)
    span = max(float(np.max(upper - lower)), 0.05)
    half_span = 0.56 * span
    return tuple(
        (float(center - half_span), float(center + half_span)) for center in centers
    )


def _style_3d_axis(axis, limits: tuple[tuple[float, float], ...]) -> None:
    axis.set_proj_type("ortho")
    axis.view_init(elev=24.0, azim=-55.0)
    axis.set_box_aspect((1.0, 1.0, 1.0))
    axis.set_xlim(*limits[0])
    axis.set_ylim(*limits[1])
    axis.set_zlim(*limits[2])
    axis.grid(True)
    for coordinate_axis in (axis.xaxis, axis.yaxis, axis.zaxis):
        coordinate_axis._axinfo["grid"]["color"] = (0.88, 0.88, 0.88, 1.0)
        coordinate_axis._axinfo["grid"]["linewidth"] = 0.45
    for pane in (axis.xaxis.pane, axis.yaxis.pane, axis.zaxis.pane):
        pane.set_facecolor((1.0, 1.0, 1.0, 0.0))
        pane.set_edgecolor(COLORS["neutral_light"])


def _plot_path_3d(
    axis,
    actual: np.ndarray,
    reference: np.ndarray | None,
    limits: tuple[tuple[float, float], ...],
    relative: bool,
    *,
    legend: bool = True,
    compact_labels: bool = False,
) -> None:
    if reference is not None:
        axis.plot(
            reference[:, 0],
            reference[:, 1],
            reference[:, 2],
            color=COLORS["reference"],
            linestyle="--",
            linewidth=1.2,
            label="Reference",
            zorder=1,
        )
    axis.plot(
        actual[:, 0],
        actual[:, 1],
        actual[:, 2],
        color=COLORS["actual"],
        linewidth=1.6,
        label="Actual",
        zorder=2,
    )
    axis.scatter(
        *actual[0],
        color=COLORS["start"],
        edgecolor="white",
        linewidth=0.5,
        s=24,
        label="Start",
        depthshade=False,
        zorder=4,
    )
    axis.scatter(
        *actual[-1],
        color=COLORS["end"],
        marker="X",
        edgecolor="white",
        linewidth=0.5,
        s=28,
        label="End",
        depthshade=False,
        zorder=5,
    )
    prefix = "$\\Delta$ " if relative else ""
    labels = AXIS_SYMBOLS if compact_labels else AXIS_NAMES
    axis.set_xlabel(f"{prefix}{labels[0]} (m)", labelpad=1 if compact_labels else 3)
    axis.set_ylabel(f"{prefix}{labels[1]} (m)", labelpad=1 if compact_labels else 3)
    if compact_labels:
        axis.set_zlabel("")
        axis.text2D(
            0.93,
            0.52,
            f"{prefix}{labels[2]} (m)",
            transform=axis.transAxes,
            rotation=90,
            ha="center",
            va="center",
        )
    else:
        axis.set_zlabel(f"{prefix}{labels[2]} (m)", labelpad=3)
    _style_3d_axis(axis, limits)
    if legend:
        axis.legend(loc="upper right", borderaxespad=0.2)


def _plot_projection(
    axis,
    actual: np.ndarray,
    reference: np.ndarray | None,
    first: int,
    second: int,
    limits: tuple[tuple[float, float], ...],
    relative: bool,
) -> None:
    if reference is not None:
        axis.plot(
            reference[:, first],
            reference[:, second],
            color=COLORS["reference"],
            linestyle="--",
            linewidth=1.0,
        )
    axis.plot(
        actual[:, first],
        actual[:, second],
        color=COLORS["actual"],
        linewidth=1.3,
    )
    axis.scatter(
        actual[0, first],
        actual[0, second],
        color=COLORS["start"],
        edgecolor="white",
        linewidth=0.4,
        s=18,
        zorder=4,
    )
    axis.scatter(
        actual[-1, first],
        actual[-1, second],
        color=COLORS["end"],
        marker="X",
        edgecolor="white",
        linewidth=0.4,
        s=22,
        zorder=5,
    )
    prefix = "$\\Delta$ " if relative else ""
    axis.set_xlabel(f"{prefix}{AXIS_NAMES[first]} (m)")
    axis.set_ylabel(f"{prefix}{AXIS_NAMES[second]} (m)")
    axis.set_xlim(*limits[first])
    axis.set_ylim(*limits[second])
    axis.set_aspect("equal", adjustable="box")
    axis.axhline(0.0, color=COLORS["neutral_light"], linewidth=0.5, zorder=0)
    axis.axvline(0.0, color=COLORS["neutral_light"], linewidth=0.5, zorder=0)


def plot_trajectory_bundle(
    output_dir: Path,
    stem: str,
    actual: Sequence[Sequence[float]],
    reference: Sequence[Sequence[float]] | None = None,
    origin: Sequence[float] | None = None,
) -> list[Path]:
    """Plot a 3D trajectory hero panel with XY, XZ, and YZ projections."""
    actual_matrix, reference_matrix, relative = _prepare_trajectory(
        actual, reference, origin
    )
    limits = _cube_limits(actual_matrix, reference_matrix)
    plt = pyplot()
    figure = plt.figure(figsize=(7.2, 4.8), constrained_layout=True)
    grid = figure.add_gridspec(3, 5, width_ratios=(1.0, 1.0, 1.0, 0.8, 0.8))
    axis_3d = figure.add_subplot(grid[:, :3], projection="3d")
    projections = (
        (figure.add_subplot(grid[0, 3:]), 0, 1, "b"),
        (figure.add_subplot(grid[1, 3:]), 0, 2, "c"),
        (figure.add_subplot(grid[2, 3:]), 1, 2, "d"),
    )
    _plot_path_3d(axis_3d, actual_matrix, reference_matrix, limits, relative)
    add_panel_label(axis_3d, "a", is_3d=True)
    for axis, first, second, panel in projections:
        _plot_projection(
            axis,
            actual_matrix,
            reference_matrix,
            first,
            second,
            limits,
            relative,
        )
        add_panel_label(axis, panel)
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def _tracking_inputs(
    times_s: Sequence[float],
    reference: Sequence[Sequence[float]],
    actual: Sequence[Sequence[float]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    reference_matrix = _as_xyz("reference series", reference)
    actual_matrix = _as_xyz("actual series", actual)
    if actual_matrix.shape != reference_matrix.shape:
        raise ValueError("reference and actual series must have matching shapes")
    times = _as_times("sample times", times_s, len(reference_matrix))
    return times, reference_matrix, actual_matrix


def _draw_tracking_axis(
    axis,
    relative_times: np.ndarray,
    reference: np.ndarray,
    actual: np.ndarray,
    component: int,
    *,
    panel: str | None = None,
    show_legend: bool = False,
    metric_y: float = 0.91,
) -> None:
    axis.plot(
        relative_times,
        reference[:, component],
        color=COLORS["reference"],
        linestyle="--",
        linewidth=1.1,
        label="Reference",
    )
    axis.plot(
        relative_times,
        actual[:, component],
        color=COLORS["actual"],
        linewidth=1.4,
        label="Actual",
    )
    axis.axhline(0.0, color=COLORS["neutral_light"], linewidth=0.6, zorder=0)
    axis.set_ylabel(f"$v_{{{AXIS_SYMBOLS[component]}}}$ (m s$^{{-1}}$)")
    rmse = float(
        np.sqrt(np.mean(np.square(actual[:, component] - reference[:, component])))
    )
    axis.text(
        0.99,
        metric_y,
        f"RMSE = {rmse:.3f} m s$^{{-1}}$",
        transform=axis.transAxes,
        ha="right",
        va="top",
        color=COLORS["neutral_dark"],
        fontsize=7.0,
    )
    if show_legend:
        axis.legend(loc="upper left", ncol=2)
    if panel is not None:
        add_panel_label(axis, panel)


def plot_velocity_tracking_bundle(
    output_dir: Path,
    stem: str,
    times_s: Sequence[float],
    reference: Sequence[Sequence[float]],
    actual: Sequence[Sequence[float]],
) -> list[Path]:
    """Plot per-axis velocity reference and actual time histories."""
    times, reference_matrix, actual_matrix = _tracking_inputs(
        times_s, reference, actual
    )
    relative = times - times[0]
    plt = pyplot()
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(7.2, 5.3),
        sharex=True,
        constrained_layout=True,
    )
    for component, axis in enumerate(axes):
        _draw_tracking_axis(
            axis,
            relative,
            reference_matrix,
            actual_matrix,
            component,
            panel=chr(ord("a") + component),
            show_legend=component == 0,
        )
    axes[-1].set_xlabel("Time (s)")
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def _draw_vector_error_axis(
    axis,
    relative_times: np.ndarray,
    errors: np.ndarray,
    ylabel: str,
    *,
    panel: str | None = None,
) -> None:
    for component, (symbol, color) in enumerate(zip(AXIS_SYMBOLS, ERROR_COLORS)):
        axis.plot(
            relative_times,
            errors[:, component],
            color=color,
            linewidth=1.1,
            label=symbol,
        )
    norm = np.linalg.norm(errors, axis=1)
    axis.plot(
        relative_times,
        norm,
        color=COLORS["error_norm"],
        linewidth=1.6,
        label="3D norm",
    )
    axis.axhline(0.0, color=COLORS["neutral_light"], linewidth=0.7, zorder=0)
    axis.set_ylabel(ylabel)
    axis.set_xlabel("Time (s)")
    rmse = float(np.sqrt(np.mean(np.square(norm))))
    axis.text(
        0.99,
        0.91,
        f"3D RMSE = {rmse:.3f}",
        transform=axis.transAxes,
        ha="right",
        va="top",
        color=COLORS["neutral_dark"],
        fontsize=7.0,
    )
    axis.legend(loc="upper left", ncol=4)
    if panel is not None:
        add_panel_label(axis, panel)


def plot_vector_error_bundle(
    output_dir: Path,
    stem: str,
    times_s: Sequence[float],
    errors: Sequence[Sequence[float]],
    ylabel: str,
) -> list[Path]:
    """Plot vector components and their Euclidean norm."""
    error_matrix = _as_xyz("vector errors", errors)
    times = _as_times("sample times", times_s, len(error_matrix))
    plt = pyplot()
    figure, axis = plt.subplots(figsize=(7.2, 3.2), constrained_layout=True)
    _draw_vector_error_axis(axis, times - times[0], error_matrix, ylabel, panel="a")
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def _draw_scalar_error_axis(
    axis,
    relative_times: np.ndarray,
    errors: np.ndarray,
    ylabel: str,
    *,
    panel: str | None = None,
) -> None:
    axis.plot(relative_times, errors, color=COLORS["error_norm"], linewidth=1.5)
    axis.axhline(0.0, color=COLORS["neutral_light"], linewidth=0.7, zorder=0)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel(ylabel)
    rmse = float(np.sqrt(np.mean(np.square(errors))))
    axis.text(
        0.99,
        0.91,
        f"RMSE = {rmse:.3f}",
        transform=axis.transAxes,
        ha="right",
        va="top",
        color=COLORS["neutral_dark"],
        fontsize=7.0,
    )
    if panel is not None:
        add_panel_label(axis, panel)


def plot_scalar_error_bundle(
    output_dir: Path,
    stem: str,
    times_s: Sequence[float],
    errors: Sequence[float],
    ylabel: str,
) -> list[Path]:
    """Plot a scalar error time history."""
    error_values = np.asarray(errors, dtype=float)
    if error_values.ndim != 1 or len(error_values) == 0:
        raise ValueError("scalar errors must be a non-empty vector")
    if not np.all(np.isfinite(error_values)):
        raise ValueError("scalar errors must contain only finite values")
    times = _as_times("sample times", times_s, len(error_values))
    plt = pyplot()
    figure, axis = plt.subplots(figsize=(7.2, 3.0), constrained_layout=True)
    _draw_scalar_error_axis(
        axis, times - times[0], error_values, ylabel, panel="a"
    )
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def _draw_norm_error_axis(
    axis,
    times: np.ndarray,
    errors: np.ndarray,
    ylabel: str,
    panel: str,
) -> None:
    relative = times - times[0]
    norm = np.linalg.norm(errors, axis=1)
    axis.plot(relative, norm, color=COLORS["error_norm"], linewidth=1.5)
    axis.axhline(0.0, color=COLORS["neutral_light"], linewidth=0.6, zorder=0)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel(ylabel)
    rmse = float(np.sqrt(np.mean(np.square(norm))))
    axis.text(
        0.99,
        0.91,
        f"RMSE = {rmse:.3f}",
        transform=axis.transAxes,
        ha="right",
        va="top",
        fontsize=7.0,
        color=COLORS["neutral_dark"],
    )
    add_panel_label(axis, panel)


def _draw_metric_summary(axis, lines: Sequence[str], panel: str) -> None:
    axis.set_axis_off()
    axis.text(
        0.0,
        1.0,
        "\n".join(lines),
        transform=axis.transAxes,
        ha="left",
        va="top",
        linespacing=1.5,
        color=COLORS["neutral_dark"],
        fontsize=7.0,
    )
    add_panel_label(axis, panel)


def _error_summary(errors: np.ndarray) -> tuple[float, float, float]:
    norm = np.linalg.norm(errors, axis=1)
    return (
        float(np.sqrt(np.mean(np.square(norm)))),
        float(np.mean(np.abs(norm))),
        float(np.max(np.abs(norm))),
    )


def plot_velocity_overview_bundle(
    output_dir: Path,
    stem: str,
    times_s: Sequence[float],
    reference: Sequence[Sequence[float]],
    actual: Sequence[Sequence[float]],
    positions: Sequence[Sequence[float]] | None,
) -> list[Path]:
    """Create the summary panel for a velocity tracking benchmark."""
    times, reference_matrix, actual_matrix = _tracking_inputs(
        times_s, reference, actual
    )
    errors = actual_matrix - reference_matrix
    relative_times = times - times[0]
    plt = pyplot()
    figure = plt.figure(figsize=(7.2, 6.2), constrained_layout=True)
    grid = figure.add_gridspec(
        4, 6, width_ratios=(1.0, 1.0, 1.0, 0.18, 1.0, 1.0)
    )
    trajectory_axis = figure.add_subplot(grid[:3, :3], projection="3d")
    if positions is not None and len(positions) >= 1:
        position_matrix = _as_xyz("base positions", positions)
        relative_position, _, _ = _prepare_trajectory(
            position_matrix, origin=position_matrix[0]
        )
        limits = _cube_limits(relative_position)
        _plot_path_3d(
            trajectory_axis,
            relative_position,
            None,
            limits,
            True,
            legend=True,
            compact_labels=True,
        )
    else:
        trajectory_axis.set_axis_off()
        trajectory_axis.text2D(
            0.5,
            0.5,
            "Base trajectory unavailable",
            transform=trajectory_axis.transAxes,
            ha="center",
            va="center",
            color=COLORS["neutral_dark"],
        )
    add_panel_label(trajectory_axis, "a", is_3d=True)
    for component in range(3):
        axis = figure.add_subplot(grid[component, 4:])
        _draw_tracking_axis(
            axis,
            relative_times,
            reference_matrix,
            actual_matrix,
            component,
            panel=chr(ord("b") + component),
            show_legend=component == 0,
            metric_y=0.72 if component == 0 else 0.91,
        )
        if component < 2:
            axis.set_xticklabels([])
        else:
            axis.set_xlabel("Time (s)")
    error_axis = figure.add_subplot(grid[3, :3])
    _draw_norm_error_axis(error_axis, times, errors, "3D velocity error (m s$^{-1}$)", "e")
    metric_axis = figure.add_subplot(grid[3, 4:])
    rmse, mae, maximum = _error_summary(errors)
    _draw_metric_summary(
        metric_axis,
        (
            f"Samples: {len(times)}",
            f"Duration: {times[-1] - times[0]:.2f} s",
            f"3D RMSE: {rmse:.3f} m s$^{{-1}}$",
            f"3D MAE: {mae:.3f} m s$^{{-1}}$",
            f"3D max: {maximum:.3f} m s$^{{-1}}$",
        ),
        "f",
    )
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def plot_base_overview_bundle(
    output_dir: Path,
    stem: str,
    times_s: Sequence[float],
    positions: Sequence[Sequence[float]],
) -> list[Path]:
    """Create the summary panel for base stability during arm motion."""
    position_matrix = _as_xyz("base positions", positions)
    times = _as_times("sample times", times_s, len(position_matrix))
    drift = position_matrix - position_matrix[0]
    limits = _cube_limits(drift)
    plt = pyplot()
    figure = plt.figure(figsize=(7.2, 4.7), constrained_layout=True)
    grid = figure.add_gridspec(
        3, 6, width_ratios=(1.0, 1.0, 1.0, 0.18, 1.0, 1.0)
    )
    trajectory_axis = figure.add_subplot(grid[:, :3], projection="3d")
    _plot_path_3d(
        trajectory_axis, drift, None, limits, True, compact_labels=True
    )
    add_panel_label(trajectory_axis, "a", is_3d=True)
    error_axis = figure.add_subplot(grid[:2, 4:])
    _draw_vector_error_axis(
        error_axis,
        times - times[0],
        drift,
        "Base drift (m)",
        panel="b",
    )
    metric_axis = figure.add_subplot(grid[2, 4:])
    rmse, mae, maximum = _error_summary(drift)
    horizontal = np.linalg.norm(drift[:, :2], axis=1)
    _draw_metric_summary(
        metric_axis,
        (
            f"Samples: {len(times)}",
            f"Duration: {times[-1] - times[0]:.2f} s",
            f"3D RMSE: {rmse:.3f} m",
            f"XY RMSE: {np.sqrt(np.mean(np.square(horizontal))):.3f} m",
            f"3D max: {maximum:.3f} m",
            f"3D MAE: {mae:.3f} m",
        ),
        "c",
    )
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def plot_aerial_overview_bundle(
    output_dir: Path,
    stem: str,
    velocity_times_s: Sequence[float],
    reference_velocity: Sequence[Sequence[float]],
    actual_velocity: Sequence[Sequence[float]],
    base_times_s: Sequence[float],
    base_positions: Sequence[Sequence[float]],
) -> list[Path]:
    """Create one summary panel for the combined aerial manipulation run."""
    velocity_times, reference_matrix, actual_matrix = _tracking_inputs(
        velocity_times_s, reference_velocity, actual_velocity
    )
    position_matrix = _as_xyz("base positions", base_positions)
    base_times = _as_times("base sample times", base_times_s, len(position_matrix))
    drift = position_matrix - position_matrix[0]
    velocity_error = actual_matrix - reference_matrix
    limits = _cube_limits(drift)
    plt = pyplot()
    figure = plt.figure(figsize=(7.2, 6.6), constrained_layout=True)
    grid = figure.add_gridspec(
        5, 6, width_ratios=(1.0, 1.0, 1.0, 0.18, 1.0, 1.0)
    )
    trajectory_axis = figure.add_subplot(grid[:3, :3], projection="3d")
    _plot_path_3d(
        trajectory_axis, drift, None, limits, True, compact_labels=True
    )
    add_panel_label(trajectory_axis, "a", is_3d=True)
    relative_velocity_times = velocity_times - velocity_times[0]
    for component in range(3):
        axis = figure.add_subplot(grid[component, 4:])
        _draw_tracking_axis(
            axis,
            relative_velocity_times,
            reference_matrix,
            actual_matrix,
            component,
            panel=chr(ord("b") + component),
            show_legend=component == 0,
            metric_y=0.72 if component == 0 else 0.91,
        )
        if component < 2:
            axis.set_xticklabels([])
        else:
            axis.set_xlabel("Time (s)")
    velocity_error_axis = figure.add_subplot(grid[3, :3])
    _draw_norm_error_axis(
        velocity_error_axis,
        velocity_times,
        velocity_error,
        "3D velocity error (m s$^{-1}$)",
        "e",
    )
    drift_axis = figure.add_subplot(grid[4, :3])
    _draw_norm_error_axis(drift_axis, base_times, drift, "3D base drift (m)", "f")
    metric_axis = figure.add_subplot(grid[3:, 4:])
    velocity_rmse, _, velocity_max = _error_summary(velocity_error)
    drift_rmse, _, drift_max = _error_summary(drift)
    _draw_metric_summary(
        metric_axis,
        (
            f"Velocity samples: {len(velocity_times)}",
            f"Velocity 3D RMSE: {velocity_rmse:.3f} m s$^{{-1}}$",
            f"Velocity 3D max: {velocity_max:.3f} m s$^{{-1}}$",
            f"Base samples: {len(base_times)}",
            f"Base 3D RMSE: {drift_rmse:.3f} m",
            f"Base 3D max: {drift_max:.3f} m",
        ),
        "g",
    )
    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths


def plot_pose_overview_bundle(
    output_dir: Path,
    stem: str,
    times_s: Sequence[float],
    reference: Sequence[Sequence[float]],
    actual: Sequence[Sequence[float]],
    velocity_times_s: Sequence[float] | None = None,
    velocity_errors: Sequence[Sequence[float]] | None = None,
    orientation_times_s: Sequence[float] | None = None,
    orientation_errors_rad: Sequence[float] | None = None,
) -> list[Path]:
    """Create the summary panel for PX4 or end-effector Figure-8 tracking."""
    times, reference_matrix, actual_matrix = _tracking_inputs(
        times_s, reference, actual
    )
    position_error = actual_matrix - reference_matrix
    limits = _cube_limits(actual_matrix, reference_matrix)
    plt = pyplot()
    figure = plt.figure(figsize=(7.2, 5.2), constrained_layout=True)
    grid = figure.add_gridspec(
        3, 6, width_ratios=(1.0, 1.0, 1.0, 0.18, 1.0, 1.0)
    )
    trajectory_axis = figure.add_subplot(grid[:, :3], projection="3d")
    _plot_path_3d(
        trajectory_axis,
        actual_matrix,
        reference_matrix,
        limits,
        False,
        compact_labels=True,
    )
    add_panel_label(trajectory_axis, "a", is_3d=True)
    position_axis = figure.add_subplot(grid[0, 4:])
    _draw_norm_error_axis(
        position_axis, times, position_error, "3D position error (m)", "b"
    )

    velocity_axis = figure.add_subplot(grid[1, 4:])
    velocity_matrix = None
    velocity_times = None
    if velocity_times_s is not None and velocity_errors is not None:
        velocity_matrix = _as_xyz("velocity errors", velocity_errors)
        velocity_times = _as_times(
            "velocity sample times", velocity_times_s, len(velocity_matrix)
        )
        _draw_norm_error_axis(
            velocity_axis,
            velocity_times,
            velocity_matrix,
            "3D velocity error (m s$^{-1}$)",
            "c",
        )
    else:
        _draw_metric_summary(velocity_axis, ("Velocity error unavailable",), "c")

    final_axis = figure.add_subplot(grid[2, 4:])
    if orientation_times_s is not None and orientation_errors_rad is not None:
        orientation_values = np.asarray(orientation_errors_rad, dtype=float)
        orientation_times = _as_times(
            "orientation sample times", orientation_times_s, len(orientation_values)
        )
        _draw_scalar_error_axis(
            final_axis,
            orientation_times - orientation_times[0],
            orientation_values,
            "Orientation error (rad)",
            panel="d",
        )
    else:
        position_rmse, _, position_max = _error_summary(position_error)
        lines = [
            f"Samples: {len(times)}",
            f"Position 3D RMSE: {position_rmse:.3f} m",
            f"Position 3D max: {position_max:.3f} m",
        ]
        if velocity_matrix is not None and velocity_times is not None:
            velocity_rmse, _, velocity_max = _error_summary(velocity_matrix)
            lines.extend(
                (
                    f"Velocity 3D RMSE: {velocity_rmse:.3f} m s$^{{-1}}$",
                    f"Velocity 3D max: {velocity_max:.3f} m s$^{{-1}}$",
                )
            )
        _draw_metric_summary(final_axis, lines, "d")

    paths = save_figure_bundle(figure, output_dir, stem)
    plt.close(figure)
    return paths
