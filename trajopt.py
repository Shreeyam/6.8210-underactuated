import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from plotting import *
from quat_helpers import *
from pydrake.all import LeafSystem, Simulator, DiagramBuilder, LogVectorOutput, MathematicalProgram, Solve, MakeSemidefiniteRelaxation

# Helper functions
def qdot_matrix(q):
    qw, qx, qy, qz = q
    return np.array([
        [-qx, -qy, -qz],
        [qw, -qz, qy],
        [qz, qw, -qx],
        [-qy, qx, qw]
    ])

def euler_eqn(w, u, J, Jinv):
    return Jinv @ (u - np.cross(w, J @ w))

def qdot(q, w):
    return 0.5 * qdot_matrix(q) @ w

def qspace(q):
    return np.array([q[4]/q[3], q[5]/q[3], q[6]/q[3]])

# Dynamics equations

# x = [wx, wy, wz, qw, qx, qy, qz]
def satellite_continuous_dynamics(x, u, J, Jinv):
    w = x[:3]
    q = x[3:]

    w_dot = euler_eqn(w, u, J, Jinv)
    q_dot = qdot(q, w)

    return np.concatenate([w_dot, q_dot])

# Implicit Euler integration for use with solver
def satellite_discrete_dynamics(x, x_next, u, time_step, J, Jinv):
    x_dot = satellite_continuous_dynamics(x_next, u, J, Jinv)
    residuals = x_next - x - time_step * x_dot
    
    return residuals


def optimize_trajectory_nonlinear(x0, xn, dt, N, J, abstol=1e-3, tau_max=None, attiude_constraint=False, avoidance_angle=40, c=None, s=None):
    prog = MathematicalProgram()

    state = prog.NewContinuousVariables(N + 1, 7, "state")
    torques = prog.NewContinuousVariables(N + 1, 3, "torques")

    Jinv = np.linalg.inv(J)

    if(avoidance_angle is not None):
        theta = np.deg2rad(avoidance_angle)
        theta = np.cos(avoidance_angle)

    if(attiude_constraint):
        A = s.dot(c.T) + c.dot(s.T) - (s.T.dot(c) + np.cos(theta)) * np.eye(3)
        b = np.cross(s.T, c.T).T
        d = s.T.dot(c) - np.cos(theta)

        Ai = np.vstack([np.hstack([d, b.T]), np.hstack([b, A])])

    for t in range(N):
        residuals = satellite_discrete_dynamics(state[t], state[t+1], torques[t], dt, J, Jinv)
        for residual in residuals:
            prog.AddConstraint(residual <= abstol)
            prog.AddConstraint(residual >= abstol)

        if(attiude_constraint):
            prog.AddConstraint(state[t, 3:].dot(Ai).dot(state[t, 3:]) <= 0)

        if(tau_max is not None):
            prog.AddConstraint(torques[t,0] <= tau_max)
            prog.AddConstraint(torques[t,1] <= tau_max)
            prog.AddConstraint(torques[t,2] <= tau_max)

            prog.AddConstraint(torques[t,0] >= -tau_max)
            prog.AddConstraint(torques[t,1] >= -tau_max)
            prog.AddConstraint(torques[t,2] >= -tau_max)

    prog.AddLinearEqualityConstraint(state[0, :], x0)
    prog.AddLinearEqualityConstraint(state[-1, :], xn)

    result = Solve(prog)
    state_opt = result.GetSolution(state)
    torques_opt = result.GetSolution(torques)

    return (result.is_success(), state_opt, torques_opt)
        

