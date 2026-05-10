from __future__ import annotations

import numpy as np


K_HALF_SQRT2 = 0.7071067811865476


def relative_seconds(timestamps: np.ndarray, start_timestamp: int) -> np.ndarray:
    return (np.asarray(timestamps, dtype=np.float64) - float(start_timestamp)) / 1e6


def ned_to_enu_positions(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    return np.column_stack((np.asarray(y, dtype=np.float64), np.asarray(x, dtype=np.float64), -np.asarray(z, dtype=np.float64)))


def quat_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=-1,
    )


def normalize_quat_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    return np.divide(q, norms, out=np.zeros_like(q), where=norms > 0.0)


def attitude_frd_to_ned_to_flu_to_enu_quat(q_frd_to_ned_wxyz: np.ndarray) -> np.ndarray:
    q_frd_to_ned_wxyz = normalize_quat_wxyz(q_frd_to_ned_wxyz)
    q_ned_to_enu = np.array([0.0, K_HALF_SQRT2, K_HALF_SQRT2, 0.0], dtype=np.float64)
    q_flu_to_frd = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
    converted = quat_multiply_wxyz(quat_multiply_wxyz(q_ned_to_enu, q_frd_to_ned_wxyz), q_flu_to_frd)
    return normalize_quat_wxyz(converted)


def quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    return np.asarray(q_wxyz, dtype=np.float64)[..., [1, 2, 3, 0]]


def rotate_vectors_by_quat_wxyz(quats_wxyz: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    quats = normalize_quat_wxyz(quats_wxyz)
    vectors = np.asarray(vectors, dtype=np.float64)
    if vectors.ndim == 1:
        vectors = np.tile(vectors, (len(quats), 1))

    q_vec = quats[:, 1:4]
    q_w = quats[:, 0:1]
    t = 2.0 * np.cross(q_vec, vectors)
    return vectors + q_w * t + np.cross(q_vec, t)


def quat_wxyz_to_euler(q_wxyz: np.ndarray) -> np.ndarray:
    q = normalize_quat_wxyz(q_wxyz)
    w, x, y, z = np.moveaxis(q, -1, 0)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.column_stack((roll, pitch, yaw))


def finite_rows(*arrays: np.ndarray) -> np.ndarray:
    if not arrays:
        return np.array([], dtype=bool)

    mask = np.ones(len(arrays[0]), dtype=bool)
    for array in arrays:
        array = np.asarray(array)
        mask &= np.isfinite(array) if array.ndim == 1 else np.isfinite(array).all(axis=1)

    return mask


def interpolate_columns(source_times: np.ndarray, values: np.ndarray, target_times: np.ndarray) -> np.ndarray:
    source_times = np.asarray(source_times, dtype=np.float64)
    target_times = np.asarray(target_times, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)

    if values.ndim == 1:
        values = values[:, None]

    result = np.empty((len(target_times), values.shape[1]), dtype=np.float64)

    for idx in range(values.shape[1]):
        finite = np.isfinite(source_times) & np.isfinite(values[:, idx])
        if not np.any(finite):
            result[:, idx] = np.nan
        else:
            result[:, idx] = np.interp(target_times, source_times[finite], values[finite, idx])

    return result
