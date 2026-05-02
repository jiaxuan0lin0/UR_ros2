import math

import numpy as np


R_HEADSET_TO_WORLD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


def normalize_vector(vector: np.ndarray, eps: float = 1.0e-12) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= eps:
        return vector.copy()
    return vector / norm


def quat_xyzw_to_matrix(quat_xyzw) -> np.ndarray:
    x, y, z, w = [float(v) for v in quat_xyzw]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def rotvec_to_matrix(rotvec) -> np.ndarray:
    vector = np.array(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle <= 1.0e-12:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def matrix_to_rotvec(matrix: np.ndarray) -> np.ndarray:
    rotation = np.array(matrix, dtype=np.float64)
    trace = float(np.trace(rotation))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle <= 1.0e-12:
        return np.zeros(3, dtype=np.float64)

    if abs(math.pi - angle) < 1.0e-5:
        axis = np.array(
            [
                math.sqrt(max(0.0, (rotation[0, 0] + 1.0) * 0.5)),
                math.sqrt(max(0.0, (rotation[1, 1] + 1.0) * 0.5)),
                math.sqrt(max(0.0, (rotation[2, 2] + 1.0) * 0.5)),
            ],
            dtype=np.float64,
        )
        axis[0] = math.copysign(axis[0], rotation[2, 1] - rotation[1, 2])
        axis[1] = math.copysign(axis[1], rotation[0, 2] - rotation[2, 0])
        axis[2] = math.copysign(axis[2], rotation[1, 0] - rotation[0, 1])
        return normalize_vector(axis) * angle

    denom = 2.0 * math.sin(angle)
    axis = np.array(
        [
            (rotation[2, 1] - rotation[1, 2]) / denom,
            (rotation[0, 2] - rotation[2, 0]) / denom,
            (rotation[1, 0] - rotation[0, 1]) / denom,
        ],
        dtype=np.float64,
    )
    return axis * angle


def pose_vector_to_parts(pose) -> tuple[np.ndarray, np.ndarray]:
    pose_array = np.array(pose, dtype=np.float64)
    return pose_array[:3], rotvec_to_matrix(pose_array[3:6])


def parts_to_pose_vector(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    pose = np.zeros(6, dtype=np.float64)
    pose[:3] = np.array(position, dtype=np.float64)
    pose[3:6] = matrix_to_rotvec(rotation)
    return pose


def transform_xr_pose(xr_pose, basis: np.ndarray = R_HEADSET_TO_WORLD) -> tuple[np.ndarray, np.ndarray]:
    pose = np.array(xr_pose, dtype=np.float64)
    position = basis @ pose[:3]
    rotation = basis @ quat_xyzw_to_matrix(pose[3:7]) @ basis.T
    return position, rotation


def scaled_rotation_delta(delta_rotation: np.ndarray, scale: float) -> np.ndarray:
    if abs(scale - 1.0) <= 1.0e-12:
        return delta_rotation
    rotvec = matrix_to_rotvec(delta_rotation) * float(scale)
    return rotvec_to_matrix(rotvec)

