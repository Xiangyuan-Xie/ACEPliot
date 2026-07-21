"""Numerical alignment and tracking metric helpers."""

from dataclasses import dataclass
from typing import Dict, Iterable, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class AlignedSeries:
    """Reference samples and interpolated measurements at common timestamps."""

    times_s: np.ndarray
    reference: np.ndarray
    actual: np.ndarray


def _as_matrix(values: Iterable[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(list(values), dtype=float)
    if matrix.ndim == 1:
        matrix = matrix.reshape((-1, 1))
    return matrix


def _clean_series(
    times_s: Iterable[float], values: Iterable[Sequence[float]]
) -> Tuple[np.ndarray, np.ndarray]:
    times = np.asarray(list(times_s), dtype=float)
    matrix = _as_matrix(values)
    if len(times) != len(matrix):
        raise ValueError("sample timestamps and values must have matching lengths")
    if len(times) == 0:
        return times, matrix

    finite = np.isfinite(times) & np.all(np.isfinite(matrix), axis=1)
    times = times[finite]
    matrix = matrix[finite]
    if len(times) == 0:
        return times, matrix

    order = np.argsort(times, kind="stable")
    times = times[order]
    matrix = matrix[order]
    keep = np.ones(len(times), dtype=bool)
    keep[:-1] = times[:-1] != times[1:]
    return times[keep], matrix[keep]


def interpolate_at(
    query_times_s: Iterable[float],
    source_times_s: Iterable[float],
    source_values: Iterable[Sequence[float]],
    max_gap_s: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Linearly interpolate source values without extrapolating across large gaps."""
    if not np.isfinite(max_gap_s) or max_gap_s <= 0.0:
        raise ValueError("max_gap_s must be finite and positive")
    queries = np.asarray(list(query_times_s), dtype=float)
    source_times, source_matrix = _clean_series(source_times_s, source_values)
    width = source_matrix.shape[1] if source_matrix.ndim == 2 else 0
    result = np.full((len(queries), width), np.nan)
    valid = np.zeros(len(queries), dtype=bool)
    if len(source_times) == 0:
        return result, valid

    for index, query in enumerate(queries):
        if not np.isfinite(query) or query < source_times[0] or query > source_times[-1]:
            continue
        right = int(np.searchsorted(source_times, query, side="left"))
        if right < len(source_times) and source_times[right] == query:
            result[index] = source_matrix[right]
            valid[index] = True
            continue
        if right == 0 or right >= len(source_times):
            continue
        left = right - 1
        span = source_times[right] - source_times[left]
        if span <= 0.0 or span > max_gap_s:
            continue
        ratio = (query - source_times[left]) / span
        result[index] = source_matrix[left] + ratio * (
            source_matrix[right] - source_matrix[left]
        )
        valid[index] = True
    return result, valid


def align_linear(
    reference_times_s: Iterable[float],
    reference_values: Iterable[Sequence[float]],
    actual_times_s: Iterable[float],
    actual_values: Iterable[Sequence[float]],
    max_gap_s: float,
) -> AlignedSeries:
    """Align measurements to reference timestamps using bounded interpolation."""
    reference_times, reference_matrix = _clean_series(
        reference_times_s, reference_values
    )
    actual_matrix, valid = interpolate_at(
        reference_times, actual_times_s, actual_values, max_gap_s
    )
    return AlignedSeries(
        times_s=reference_times[valid],
        reference=reference_matrix[valid],
        actual=actual_matrix[valid],
    )


def interpolate_quaternions_at(
    query_times_s: Iterable[float],
    source_times_s: Iterable[float],
    source_wxyz: Iterable[Sequence[float]],
    max_gap_s: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Interpolate unit quaternions with sign-aware normalized lerp."""
    if not np.isfinite(max_gap_s) or max_gap_s <= 0.0:
        raise ValueError("max_gap_s must be finite and positive")
    queries = np.asarray(list(query_times_s), dtype=float)
    source_times, quaternions = _clean_series(source_times_s, source_wxyz)
    result = np.full((len(queries), 4), np.nan)
    valid = np.zeros(len(queries), dtype=bool)
    if len(source_times) == 0:
        return result, valid
    if quaternions.shape[1:] != (4,):
        raise ValueError("source_wxyz must contain four-value quaternions")
    norms = np.linalg.norm(quaternions, axis=1)
    quaternion_valid = norms > 1.0e-12
    quaternions[quaternion_valid] /= norms[quaternion_valid, None]
    for index, query in enumerate(queries):
        if not np.isfinite(query) or query < source_times[0] or query > source_times[-1]:
            continue
        right = int(np.searchsorted(source_times, query, side="left"))
        if right < len(source_times) and source_times[right] == query:
            if quaternion_valid[right]:
                result[index] = quaternions[right]
                valid[index] = True
            continue
        if right == 0 or right >= len(source_times):
            continue
        left = right - 1
        span = source_times[right] - source_times[left]
        if (
            span <= 0.0
            or span > max_gap_s
            or not quaternion_valid[left]
            or not quaternion_valid[right]
        ):
            continue
        start = quaternions[left]
        end = quaternions[right]
        if np.dot(start, end) < 0.0:
            end = -end
        ratio = (query - source_times[left]) / span
        interpolated = start + ratio * (end - start)
        norm = np.linalg.norm(interpolated)
        if norm > 1.0e-12:
            result[index] = interpolated / norm
            valid[index] = True
    return result, valid


def tracking_metrics(
    reference: np.ndarray, actual: np.ndarray, axis_names: Sequence[str]
) -> Dict[str, object]:
    """Compute per-axis and vector RMSE, MAE, and maximum absolute error."""
    reference = np.asarray(reference, dtype=float)
    actual = np.asarray(actual, dtype=float)
    if reference.shape != actual.shape or reference.ndim != 2:
        raise ValueError("reference and actual values must be matching matrices")
    if reference.shape[1] != len(axis_names):
        raise ValueError("axis_names length must match matrix width")
    if len(reference) == 0:
        raise ValueError("at least one sample is required")

    error = actual - reference
    axes: Dict[str, Dict[str, float]] = {}
    for axis, column in zip(axis_names, error.T):
        axes[axis] = {
            "rmse": float(np.sqrt(np.mean(np.square(column)))),
            "mae": float(np.mean(np.abs(column))),
            "max_abs": float(np.max(np.abs(column))),
        }
    norm = np.linalg.norm(error, axis=1)
    result = {
        "axes": axes,
        "vector": {
            "rmse": float(np.sqrt(np.mean(np.square(norm)))),
            "mae": float(np.mean(norm)),
            "max": float(np.max(norm)),
        },
    }
    if error.shape[1] >= 2:
        horizontal = np.linalg.norm(error[:, :2], axis=1)
        result["horizontal"] = {
            "rmse": float(np.sqrt(np.mean(np.square(horizontal)))),
            "mae": float(np.mean(horizontal)),
            "max": float(np.max(horizontal)),
        }
    return result


def base_drift_metrics(positions_enu_m: np.ndarray) -> Dict[str, object]:
    """Compute hover drift relative to the first position sample."""
    positions = np.asarray(positions_enu_m, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) == 0:
        raise ValueError("positions must be a non-empty Nx3 matrix")
    reference = positions[0]
    drift = positions - reference
    metrics = tracking_metrics(np.zeros_like(drift), drift, ("x", "y", "z"))
    horizontal = np.linalg.norm(drift[:, :2], axis=1)
    spatial = np.linalg.norm(drift, axis=1)
    metrics["reference_position_enu_m"] = reference.tolist()
    metrics["horizontal"] = {
        "rmse": float(np.sqrt(np.mean(np.square(horizontal)))),
        "mae": float(np.mean(horizontal)),
        "max": float(np.max(horizontal)),
    }
    metrics["spatial"] = {
        "rmse": float(np.sqrt(np.mean(np.square(spatial)))),
        "mae": float(np.mean(spatial)),
        "max": float(np.max(spatial)),
    }
    return metrics


def derive_velocity(times_s: np.ndarray, positions: np.ndarray) -> np.ndarray:
    """Differentiate positions using timestamp-aware finite differences."""
    times = np.asarray(times_s, dtype=float)
    values = np.asarray(positions, dtype=float)
    if len(times) < 2 or values.shape != (len(times), 3):
        raise ValueError("at least two timestamped 3D positions are required")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("timestamps must be strictly increasing")
    return np.column_stack(
        [np.gradient(values[:, axis], times, edge_order=1) for axis in range(3)]
    )


def quaternion_angular_errors(
    reference_wxyz: np.ndarray, actual_wxyz: np.ndarray
) -> np.ndarray:
    """Return shortest geodesic quaternion errors in radians."""
    reference = np.asarray(reference_wxyz, dtype=float)
    actual = np.asarray(actual_wxyz, dtype=float)
    if reference.shape != actual.shape or reference.ndim != 2 or reference.shape[1] != 4:
        raise ValueError("quaternion arrays must be matching Nx4 matrices")
    reference /= np.linalg.norm(reference, axis=1, keepdims=True)
    actual /= np.linalg.norm(actual, axis=1, keepdims=True)
    dots = np.clip(np.abs(np.sum(reference * actual, axis=1)), 0.0, 1.0)
    return 2.0 * np.arccos(dots)


def scalar_error_metrics(errors: np.ndarray) -> Dict[str, float]:
    """Compute scalar RMSE, MAE, and maximum absolute error."""
    values = np.asarray(errors, dtype=float)
    if len(values) == 0:
        raise ValueError("at least one error sample is required")
    return {
        "rmse": float(np.sqrt(np.mean(np.square(values)))),
        "mae": float(np.mean(np.abs(values))),
        "max_abs": float(np.max(np.abs(values))),
    }
