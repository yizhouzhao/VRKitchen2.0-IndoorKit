import numpy as np

def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    return q_r[:, 0:3] * np.sign(q_r[:, 3])[:, None]

def quat_mul(a, b):
    assert a.shape == b.shape
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    x1, y1, z1, w1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    x2, y2, z2, w2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = np.stack([x, y, z, w], axis=-1).reshape(shape)

    return quat


def normalize(x, eps: float = 1e-9):
    return x / np.clip(np.linalg.norm(x, axis=-1), a_min=eps, a_max=None)[:, None]


def quat_unit(a):
    return normalize(a)


def quat_from_angle_axis(angle, axis):
    theta = (angle / 2)[:, None]
    xyz = normalize(axis) * np.sin(theta)
    w = np.cos(theta)
    return quat_unit(np.concatenate([xyz, w], axis=-1))


def quat_rotate(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0)[:, None]
    b = np.cross(q_vec, v) * q_w[:, None] * 2.0
    c = q_vec * np.sum(q_vec * v, axis=1).reshape(shape[0], -1) * 2.0
    return a + b + c


def quat_conjugate(a):
    shape = a.shape
    a = a.reshape(-1, 4)
    return np.concatenate((-a[:, :3], a[:, -1:]), axis=-1).reshape(shape)


def quat_axis(q, axis=0):
    basis_vec = np.zeros((q.shape[0], 3))
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)
