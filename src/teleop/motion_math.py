import numpy as np


def scale_to_component_limit(vector: np.ndarray, component_limit: float) -> np.ndarray:
    values = np.array(vector, dtype=np.float64)
    limit = abs(float(component_limit))
    max_abs = float(np.max(np.abs(values))) if values.size else 0.0
    if limit > 0.0 and max_abs > limit:
        values *= limit / max_abs
    return values


def limit_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    values = np.array(vector, dtype=np.float64)
    limit = abs(float(max_norm))
    norm = float(np.linalg.norm(values))
    if limit > 0.0 and norm > limit:
        values *= limit / norm
    return values


def apply_deadband(vector: np.ndarray, deadband: float) -> np.ndarray:
    values = np.array(vector, dtype=np.float64)
    threshold = max(0.0, float(deadband))
    norm = float(np.linalg.norm(values))
    if threshold <= 0.0 or norm <= 0.0:
        return values
    if norm <= threshold:
        return np.zeros_like(values)
    return values * ((norm - threshold) / norm)


def nearest_joint_equivalent(target_q, reference_q) -> np.ndarray:
    target = np.array(target_q, dtype=np.float64)
    reference = np.array(reference_q, dtype=np.float64)
    delta = (target - reference + np.pi) % (2.0 * np.pi) - np.pi
    return reference + delta


def dls_joint_velocity(
    jacobian: np.ndarray,
    twist: np.ndarray,
    damping: float,
    max_damping: float,
    singularity_threshold: float,
    hard_stop_threshold: float,
):
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    min_singular = float(np.min(singular_values))
    hard_stop = max(0.0, float(hard_stop_threshold))
    soft_stop = max(hard_stop + 1.0e-6, float(singularity_threshold))

    if min_singular >= soft_stop:
        singular_scale = 1.0
        singular_ratio = 0.0
    elif min_singular <= hard_stop:
        singular_scale = 0.0
        singular_ratio = 1.0
    else:
        singular_scale = (min_singular - hard_stop) / (soft_stop - hard_stop)
        singular_ratio = 1.0 - singular_scale

    base_damping = max(0.0, float(damping))
    max_damping = max(base_damping, float(max_damping))
    damping = base_damping + (max_damping - base_damping) * singular_ratio * singular_ratio

    scaled_twist = np.array(twist, dtype=np.float64) * singular_scale
    regularizer = (damping * damping) * np.eye(6, dtype=np.float64)
    try:
        qdot = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + regularizer,
            scaled_twist,
        )
    except np.linalg.LinAlgError:
        qdot = np.zeros(6, dtype=np.float64)
    if not np.all(np.isfinite(qdot)):
        qdot = np.zeros(6, dtype=np.float64)
    return qdot, min_singular, singular_scale, damping
