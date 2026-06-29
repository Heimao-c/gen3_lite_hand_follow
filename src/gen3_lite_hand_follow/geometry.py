#!/usr/bin/env python3
import math


def clamp(value, low, high):
    return max(low, min(high, value))


def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def clamp_norm(v, max_norm):
    n = norm3(v)
    if n <= max_norm or n < 1.0e-9:
        return list(v)
    scale = max_norm / n
    return [v[0] * scale, v[1] * scale, v[2] * scale]


def deadband(v, threshold):
    if norm3(v) < threshold:
        return [0.0, 0.0, 0.0]
    return list(v)


def quat_to_matrix(q):
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rotate_vector(q_xyzw, v):
    r = quat_to_matrix(q_xyzw)
    return [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]


def transform_to_xyz_quat(transform_stamped):
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
