import numpy as np
import torch


@torch.jit.script
def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate a vector by the inverse of a quaternion.

    Args:
        q: The quaternion in (w, x, y, z). Shape is (N, 4).
        v: The vector in (x, y, z). Shape is (N, 3).

    Returns:
        The rotated vector in (x, y, z). Shape is (N, 3).
    """
    shape = q.shape
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w**2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def quaternion_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quaternion_to_gravity_component(quaternion: np.ndarray):
    # Assuming quaternion as (w, x, y, z)
    gravity_vector_global = np.array([0, 0, -1])

    # Convert gravity vector to a quaternion
    gravity_quaternion_global = np.array([0, *gravity_vector_global])

    # Rotate the gravity quaternion by the object's quaternion
    rotated_gravity_quaternion = quaternion_multiply(
        quaternion, quaternion_multiply(gravity_quaternion_global, quaternion_conjugate(quaternion))
    )

    # Extract the result as a numpy array
    gravity_component_local = np.array(
        [rotated_gravity_quaternion[1], rotated_gravity_quaternion[2], rotated_gravity_quaternion[3]]
    )

    return gravity_component_local
