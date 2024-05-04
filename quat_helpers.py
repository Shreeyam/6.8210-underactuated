import numpy as np

def qconj(q):
    p = q.copy()
    p[1:] *= -1
    return p

def qmult(q1, q2):
    w1, v1 = q1[0], q1[1:]
    w2, v2 = q2[0], q2[1:]
    # Use geometric product to multiply quaternions
    w = w1*w2 - np.dot(v1, v2)
    v = w1*v2 + w2*v1 + np.cross(v1, v2)
    return np.hstack([w, v])

def qsandwich(q, v):
    p = np.hstack([0, v])
    return qmult(q, qmult(p, qconj(q)))[1:]

def qarray(q, func, *args):
    return np.array([func(qi, *args) for qi in q])